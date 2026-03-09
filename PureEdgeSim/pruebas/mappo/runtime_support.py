from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]
DEFAULT_CONFIG_PATH = SCRIPT_DIR / "runtime_config.json"

_INT_KEYS = {"port", "episodes", "save_interval", "progress_log_interval"}
_ENV_OVERRIDES = {
    "python_exe": "PUREEDGESIM_MAPPO_PYTHON_EXE",
    "java_launch_cmd": "PUREEDGESIM_MAPPO_JAVA_LAUNCH_CMD",
    "java_main_class": "PUREEDGESIM_MAPPO_JAVA_MAIN_CLASS",
    "host": "PUREEDGESIM_MAPPO_HOST",
    "port": "PUREEDGESIM_MAPPO_PORT",
    "train_settings_dir": "PUREEDGESIM_MAPPO_TRAIN_SETTINGS_DIR",
    "eval_settings_dir": "PUREEDGESIM_MAPPO_EVAL_SETTINGS_DIR",
    "episodes": "PUREEDGESIM_MAPPO_EPISODES",
    "save_interval": "PUREEDGESIM_MAPPO_SAVE_INTERVAL",
    "progress_log_interval": "PUREEDGESIM_MAPPO_PROGRESS_LOG_INTERVAL",
    "model_dir": "PUREEDGESIM_MAPPO_MODEL_DIR",
    "output_root": "PUREEDGESIM_MAPPO_OUTPUT_ROOT",
}


@dataclass
class RuntimeConfig:
    python_exe: str
    java_launch_cmd: List[str]
    java_main_class: str
    host: str
    port: int
    train_settings_dir: Path
    eval_settings_dir: Path
    episodes: int
    save_interval: int
    progress_log_interval: int
    model_dir: Path
    output_root: Path

    def to_dict(self) -> Dict[str, Any]:
        return {
            "python_exe": self.python_exe,
            "java_launch_cmd": list(self.java_launch_cmd),
            "java_main_class": self.java_main_class,
            "host": self.host,
            "port": self.port,
            "train_settings_dir": str(self.train_settings_dir),
            "eval_settings_dir": str(self.eval_settings_dir),
            "episodes": self.episodes,
            "save_interval": self.save_interval,
            "progress_log_interval": self.progress_log_interval,
            "model_dir": str(self.model_dir),
            "output_root": str(self.output_root),
        }


class JavaEpisodeProcess:
    def __init__(self, process: subprocess.Popen, label: str) -> None:
        self.process = process
        self.label = label
        self.tail: Deque[str] = deque(maxlen=50)
        self._thread = threading.Thread(target=self._stream_output, name=f"{label}-stdout", daemon=True)
        self._thread.start()

    def _stream_output(self) -> None:
        if self.process.stdout is None:
            return
        for raw_line in self.process.stdout:
            line = raw_line.rstrip()
            self.tail.append(line)
            print(f"[{self.label}] {line}", flush=True)

    def poll(self) -> Optional[int]:
        return self.process.poll()

    def wait(self, timeout: Optional[float] = None) -> int:
        return self.process.wait(timeout=timeout)

    def terminate(self) -> None:
        if self.poll() is not None:
            return
        self.process.terminate()
        try:
            self.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.wait(timeout=5)

    def ensure_success(self) -> None:
        return_code = self.poll()
        if return_code is None:
            return
        if return_code != 0:
            raise RuntimeError(
                f"Java simulation exited with code {return_code}.\nRecent output:\n{self.recent_output()}"
            )

    def recent_output(self) -> str:
        if not self.tail:
            return "<no java output captured>"
        return "\n".join(self.tail)


def load_config(config_path: Optional[Path] = None) -> RuntimeConfig:
    path = config_path or Path(os.getenv("PUREEDGESIM_MAPPO_CONFIG", DEFAULT_CONFIG_PATH))
    if not path.is_absolute():
        path = REPO_ROOT / path

    with path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)

    raw = _apply_env_overrides(raw)
    required = {
        "python_exe",
        "java_launch_cmd",
        "java_main_class",
        "host",
        "port",
        "train_settings_dir",
        "eval_settings_dir",
        "episodes",
        "save_interval",
        "progress_log_interval",
        "model_dir",
        "output_root",
    }
    missing = sorted(required.difference(raw.keys()))
    if missing:
        raise KeyError(f"Missing MAPPO runtime config keys: {', '.join(missing)}")

    java_launch_cmd = raw["java_launch_cmd"]
    if isinstance(java_launch_cmd, str):
        java_launch_cmd = shlex.split(java_launch_cmd, posix=os.name != "nt")
    if not isinstance(java_launch_cmd, list) or not java_launch_cmd:
        raise ValueError("runtime_config.json key 'java_launch_cmd' must be a non-empty list or string.")

    return RuntimeConfig(
        python_exe=str(raw["python_exe"]),
        java_launch_cmd=[str(part) for part in java_launch_cmd],
        java_main_class=str(raw["java_main_class"]),
        host=str(raw["host"]),
        port=int(raw["port"]),
        train_settings_dir=_resolve_path(raw["train_settings_dir"]),
        eval_settings_dir=_resolve_path(raw["eval_settings_dir"]),
        episodes=max(1, int(raw["episodes"])),
        save_interval=max(1, int(raw["save_interval"])),
        progress_log_interval=max(1, int(raw["progress_log_interval"])),
        model_dir=_resolve_path(raw["model_dir"]),
        output_root=_resolve_path(raw["output_root"]),
    )


def describe_runtime(config: RuntimeConfig) -> None:
    print(f"running python: {sys.executable}", flush=True)
    if config.python_exe:
        print(f"config python_exe: {config.python_exe}", flush=True)


def compile_java_project(config: RuntimeConfig) -> None:
    compile_cmd = _resolve_launch_cmd(list(config.java_launch_cmd)) + ["compile"]
    print(f"compiling java: {' '.join(compile_cmd)}", flush=True)
    subprocess.run(compile_cmd, cwd=REPO_ROOT, check=True)


def start_java_episode(
    config: RuntimeConfig,
    settings_dir: Path,
    output_dir: Path,
    label: str,
) -> JavaEpisodeProcess:
    output_dir.mkdir(parents=True, exist_ok=True)
    cmd = _resolve_launch_cmd(list(config.java_launch_cmd))
    cmd.extend(
        [
            "-Dmappo.env.server=true",
            f"-Dmappo.env.port={config.port}",
            f"-DsettingsPath={with_trailing_separator(settings_dir)}",
            f"-DoutputPath={with_trailing_separator(output_dir)}",
            f"-Dexec.mainClass={config.java_main_class}",
            "exec:java",
        ]
    )
    print(f"starting java episode: {' '.join(cmd)}", flush=True)
    process = subprocess.Popen(
        cmd,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    return JavaEpisodeProcess(process, label)


def connect_client_with_retry(
    client: Any,
    process: JavaEpisodeProcess,
    timeout_s: float = 30.0,
    retry_interval_s: float = 0.25,
) -> None:
    deadline = time.time() + timeout_s
    last_error: Optional[BaseException] = None

    while time.time() < deadline:
        process.ensure_success()
        try:
            client.connect()
            return
        except OSError as exc:
            last_error = exc
            time.sleep(retry_interval_s)

    process.ensure_success()
    raise TimeoutError(
        f"Timed out after {timeout_s:.1f}s waiting for MAPPOEnvServer at {client.host}:{client.port}. "
        f"Last error: {last_error}"
    )


def wait_for_java_exit(process: JavaEpisodeProcess, timeout_s: float = 60.0) -> None:
    try:
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired as exc:
        process.terminate()
        raise TimeoutError(
            f"Java simulation did not exit within {timeout_s:.1f}s.\nRecent output:\n{process.recent_output()}"
        ) from exc
    process.ensure_success()


def build_output_dir(config: RuntimeConfig, mode: str, episode_index: int) -> Path:
    return config.output_root / mode / f"episode_{episode_index:03d}"


def resolve_model_path(config: RuntimeConfig, name: str = "latest.pt") -> Path:
    config.model_dir.mkdir(parents=True, exist_ok=True)
    return config.model_dir / name


def with_trailing_separator(path: Path) -> str:
    normalized = path.resolve().as_posix().rstrip("/")
    return normalized + "/"


def _apply_env_overrides(raw: Dict[str, Any]) -> Dict[str, Any]:
    merged = dict(raw)
    for key, env_name in _ENV_OVERRIDES.items():
        value = os.getenv(env_name)
        if value is None or value == "":
            continue
        if key == "java_launch_cmd":
            merged[key] = shlex.split(value, posix=os.name != "nt")
        elif key in _INT_KEYS:
            merged[key] = int(value)
        else:
            merged[key] = value
    return merged


def _resolve_path(value: Any) -> Path:
    path = Path(str(value))
    if not path.is_absolute():
        path = REPO_ROOT / path
    return path


def _resolve_launch_cmd(cmd: List[str]) -> List[str]:
    if not cmd:
        raise ValueError("java_launch_cmd cannot be empty.")

    resolved = shutil.which(cmd[0])
    if resolved is None and os.name == "nt" and not cmd[0].lower().endswith(".cmd"):
        resolved = shutil.which(cmd[0] + ".cmd")
    if resolved is not None:
        cmd[0] = resolved
    return cmd
