from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


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
    "settings_dir": "PUREEDGESIM_MAPPO_SETTINGS_DIR",
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
    settings_dir: Path
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
            "settings_dir": str(self.settings_dir),
            "episodes": self.episodes,
            "save_interval": self.save_interval,
            "progress_log_interval": self.progress_log_interval,
            "model_dir": str(self.model_dir),
            "output_root": str(self.output_root),
        }


class RunLogger:
    def __init__(self, log_path: Path) -> None:
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._file = self.log_path.open("a", encoding="utf-8", buffering=1)

    def log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with self._lock:
            self._file.write(f"{timestamp} {message}\n")

    def open_subprocess_stream(self):
        return self.log_path.open("a", encoding="utf-8", buffering=1)

    def close(self) -> None:
        with self._lock:
            if not self._file.closed:
                self._file.close()


class JavaEpisodeProcess:
    def __init__(self, process: subprocess.Popen, label: str, logger: RunLogger) -> None:
        self.process = process
        self.label = label
        self.logger = logger
        self.log_path = logger.log_path
        self._tail: List[str] = []
        self._tail_lock = threading.Lock()
        self._thread = threading.Thread(target=self._stream_output, name=f"{label}-stdout", daemon=True)
        self._thread.start()

    def _stream_output(self) -> None:
        if self.process.stdout is None:
            return
        for raw_line in self.process.stdout:
            line = raw_line.rstrip()
            self.logger.log(f"[{self.label}] {line}")
            with self._tail_lock:
                self._tail.append(line)
                if len(self._tail) > 50:
                    self._tail.pop(0)

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
                f"Java simulation exited with code {return_code}.\n"
                f"Recent output:\n{self.recent_output()}\n"
                f"Full log: {self.log_path}"
            )

    def recent_output(self) -> str:
        with self._tail_lock:
            if not self._tail:
                return "<no java output captured>"
            return "\n".join(self._tail)


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

    settings_dir = _resolve_shared_settings_dir(raw)

    return RuntimeConfig(
        python_exe=str(raw["python_exe"]),
        java_launch_cmd=[str(part) for part in java_launch_cmd],
        java_main_class=str(raw["java_main_class"]),
        host=str(raw["host"]),
        port=int(raw["port"]),
        settings_dir=settings_dir,
        episodes=max(1, int(raw["episodes"])),
        save_interval=max(1, int(raw["save_interval"])),
        progress_log_interval=max(1, int(raw["progress_log_interval"])),
        model_dir=_resolve_path(raw["model_dir"]),
        output_root=_resolve_path(raw["output_root"]),
    )


def create_run_logger(config: RuntimeConfig, mode: str, file_prefix: str) -> RunLogger:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = config.output_root / mode / "logs" / f"{file_prefix}_{timestamp}.log"
    logger = RunLogger(log_path)
    logger.log(f"run_logger_initialized mode={mode}")
    return logger


def describe_runtime(config: RuntimeConfig, logger: RunLogger) -> None:
    logger.log(f"python_executable={config.python_exe}")
    logger.log(f"runtime_config={json.dumps(config.to_dict(), ensure_ascii=True)}")


def compile_java_project(config: RuntimeConfig, logger: RunLogger) -> None:
    compile_cmd = _resolve_launch_cmd(list(config.java_launch_cmd)) + ["compile"]
    logger.log(f"compile_command={' '.join(compile_cmd)}")
    with logger.open_subprocess_stream() as sink:
        result = subprocess.run(
            compile_cmd,
            cwd=REPO_ROOT,
            stdout=sink,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )
    if result.returncode != 0:
        raise RuntimeError(f"Java compile failed with code {result.returncode}. See log: {logger.log_path}")


def start_java_episode(
    config: RuntimeConfig,
    settings_dir: Path,
    output_dir: Path,
    label: str,
    logger: RunLogger,
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
    logger.log(f"start_java_episode label={label} command={' '.join(cmd)}")
    process = subprocess.Popen(
        cmd,
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    return JavaEpisodeProcess(process, label, logger)


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
        f"Last error: {last_error}\nFull log: {process.log_path}"
    )


def wait_for_java_exit(process: JavaEpisodeProcess, timeout_s: float = 60.0) -> None:
    try:
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired as exc:
        process.terminate()
        raise TimeoutError(
            f"Java simulation did not exit within {timeout_s:.1f}s.\n"
            f"Recent output:\n{process.recent_output()}\n"
            f"Full log: {process.log_path}"
        ) from exc
    process.ensure_success()


def prepare_effective_settings_dir(
    config: RuntimeConfig,
    base_settings_dir: Path,
    mode: str,
    run_id: str,
    simulation_minutes_override: Optional[int],
    logger: RunLogger,
    display_real_time_charts_override: Optional[bool] = None,
    auto_close_real_time_charts_override: Optional[bool] = None,
) -> tuple[Path, int]:
    base_settings_dir = base_settings_dir.resolve()
    if (
        simulation_minutes_override is None
        and display_real_time_charts_override is None
        and auto_close_real_time_charts_override is None
    ):
        simulation_minutes = read_simulation_minutes(base_settings_dir)
        display_real_time_charts = read_boolean_setting(base_settings_dir, "display_real_time_charts")
        auto_close_real_time_charts = read_boolean_setting(base_settings_dir, "auto_close_real_time_charts")
        logger.log(
            f"using_base_settings_dir mode={mode} settings_dir={base_settings_dir} "
            f"simulation_minutes={simulation_minutes} "
            f"display_real_time_charts={str(display_real_time_charts).lower()} "
            f"auto_close_real_time_charts={str(auto_close_real_time_charts).lower()}"
        )
        return base_settings_dir, simulation_minutes

    overrides: Dict[str, str] = {}
    if simulation_minutes_override is not None:
        overrides["simulation_time"] = str(max(1, int(simulation_minutes_override)))
    if display_real_time_charts_override is not None:
        overrides["display_real_time_charts"] = str(display_real_time_charts_override).lower()
    if auto_close_real_time_charts_override is not None:
        overrides["auto_close_real_time_charts"] = str(auto_close_real_time_charts_override).lower()
    runtime_settings_dir = clone_settings_dir(config, base_settings_dir, mode, run_id)
    write_settings_overrides(runtime_settings_dir, overrides)
    simulation_minutes = read_simulation_minutes(runtime_settings_dir)
    display_real_time_charts = read_boolean_setting(runtime_settings_dir, "display_real_time_charts")
    auto_close_real_time_charts = read_boolean_setting(runtime_settings_dir, "auto_close_real_time_charts")
    logger.log(
        f"created_runtime_settings_dir mode={mode} base_settings_dir={base_settings_dir} "
        f"runtime_settings_dir={runtime_settings_dir} simulation_minutes={simulation_minutes} "
        f"display_real_time_charts={str(display_real_time_charts).lower()} "
        f"auto_close_real_time_charts={str(auto_close_real_time_charts).lower()}"
    )
    return runtime_settings_dir, simulation_minutes


def prepare_stress_settings_dir(
    config: RuntimeConfig,
    base_settings_dir: Path,
    mode: str,
    run_id: str,
    logger: RunLogger,
    simulation_minutes_override: Optional[int] = None,
    random_seed_override: Optional[int] = None,
    edge_datacenters_coverage: int = 110,
    application_rate_scale: float = 1.5,
    display_real_time_charts_override: Optional[bool] = None,
    auto_close_real_time_charts_override: Optional[bool] = None,
) -> tuple[Path, int]:
    runtime_settings_dir = clone_settings_dir(config, base_settings_dir.resolve(), mode, run_id)
    overrides: Dict[str, str] = {
        "edge_datacenters_coverage": str(int(edge_datacenters_coverage)),
    }
    if simulation_minutes_override is not None:
        overrides["simulation_time"] = str(max(1, int(simulation_minutes_override)))
    if random_seed_override is not None:
        overrides["random_seed"] = str(int(random_seed_override))
    if display_real_time_charts_override is not None:
        overrides["display_real_time_charts"] = str(display_real_time_charts_override).lower()
    if auto_close_real_time_charts_override is not None:
        overrides["auto_close_real_time_charts"] = str(auto_close_real_time_charts_override).lower()

    write_settings_overrides(runtime_settings_dir, overrides)
    scale_application_rates(runtime_settings_dir, application_rate_scale)

    simulation_minutes = read_simulation_minutes(runtime_settings_dir)
    logger.log(
        "created_stress_settings_dir "
        f"base_settings_dir={base_settings_dir.resolve()} "
        f"runtime_settings_dir={runtime_settings_dir} "
        f"simulation_minutes={simulation_minutes} "
        f"random_seed={random_seed_override} "
        f"edge_datacenters_coverage={edge_datacenters_coverage} "
        f"application_rate_scale={application_rate_scale}"
    )
    return runtime_settings_dir, simulation_minutes


def build_output_dir(config: RuntimeConfig, mode: str, episode_index: int) -> Path:
    return config.output_root / mode / f"episode_{episode_index:03d}"


def resolve_model_path(config: RuntimeConfig, name: str = "latest.pt") -> Path:
    config.model_dir.mkdir(parents=True, exist_ok=True)
    return config.model_dir / name


def clone_settings_dir(config: RuntimeConfig, base_settings_dir: Path, mode: str, run_id: str) -> Path:
    runtime_settings_dir = config.output_root / mode / "runtime_settings" / run_id
    if runtime_settings_dir.exists():
        shutil.rmtree(runtime_settings_dir)
    runtime_settings_dir.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(base_settings_dir, runtime_settings_dir)
    return runtime_settings_dir


def with_trailing_separator(path: Path) -> str:
    normalized = path.resolve().as_posix().rstrip("/")
    return normalized + "/"


def read_simulation_minutes(settings_dir: Path) -> int:
    sim_file = settings_dir / "simulation_parameters.properties"
    for line in sim_file.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("simulation_time="):
            return max(1, int(float(stripped.split("=", 1)[1].strip())))
    raise ValueError(f"simulation_time not found in {sim_file}")


def read_boolean_setting(settings_dir: Path, key: str) -> bool:
    sim_file = settings_dir / "simulation_parameters.properties"
    prefix = f"{key}="
    for line in sim_file.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith(prefix):
            value = stripped.split("=", 1)[1].strip().lower()
            if value == "true":
                return True
            if value == "false":
                return False
            raise ValueError(f"{key} must be true or false in {sim_file}, found: {value}")
    raise ValueError(f"{key} not found in {sim_file}")


def write_simulation_minutes(settings_dir: Path, simulation_minutes: int) -> None:
    write_settings_overrides(settings_dir, {"simulation_time": str(max(1, int(simulation_minutes)))})


def write_settings_overrides(settings_dir: Path, overrides: Dict[str, str]) -> None:
    if not overrides:
        return
    sim_file = settings_dir / "simulation_parameters.properties"
    updated_lines = []
    replaced = set()
    for line in sim_file.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        matched_key = None
        for key in overrides:
            if stripped.startswith(f"{key}="):
                matched_key = key
                break
        if matched_key is None:
            updated_lines.append(line)
            continue
        updated_lines.append(f"{matched_key}={overrides[matched_key]}")
        replaced.add(matched_key)
    missing = sorted(set(overrides).difference(replaced))
    if missing:
        raise ValueError(f"Missing settings in {sim_file}: {', '.join(missing)}")
    sim_file.write_text("\n".join(updated_lines) + "\n", encoding="utf-8")


def scale_application_rates(settings_dir: Path, scale: float) -> None:
    if scale <= 0:
        raise ValueError(f"application rate scale must be positive, got {scale}")
    app_file = settings_dir / "applications.xml"
    tree = ET.parse(app_file)
    root = tree.getroot()
    changed = False
    for app in root.findall("application"):
        rate_el = app.find("rate")
        if rate_el is None or rate_el.text is None:
            continue
        old_rate = float(rate_el.text.strip())
        new_rate = max(1, int(round(old_rate * scale)))
        rate_el.text = str(new_rate)
        changed = True
    if not changed:
        raise ValueError(f"No application rates found in {app_file}")
    tree.write(app_file, encoding="utf-8", xml_declaration=True)


def _resolve_shared_settings_dir(raw: Dict[str, Any]) -> Path:
    settings_dir = raw.get("settings_dir")
    if settings_dir is not None and str(settings_dir).strip() != "":
        return _resolve_path(settings_dir)

    train_settings_dir = raw.get("train_settings_dir")
    eval_settings_dir = raw.get("eval_settings_dir")
    if train_settings_dir is not None and eval_settings_dir is not None:
        train_path = _resolve_path(train_settings_dir)
        eval_path = _resolve_path(eval_settings_dir)
        if train_path != eval_path:
            raise ValueError(
                "Legacy MAPPO config keys 'train_settings_dir' and 'eval_settings_dir' must match "
                f"when 'settings_dir' is absent, got '{train_path}' and '{eval_path}'."
            )
        return train_path
    if train_settings_dir is not None:
        return _resolve_path(train_settings_dir)
    if eval_settings_dir is not None:
        return _resolve_path(eval_settings_dir)
    raise KeyError("Missing MAPPO runtime config key: settings_dir")


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
