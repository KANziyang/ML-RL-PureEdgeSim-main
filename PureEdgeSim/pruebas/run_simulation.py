from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_OUTPUT_ROOT = SCRIPT_DIR / "output_builtin"
DEFAULT_SETTINGS_DIR = SCRIPT_DIR / "settings_base"
DEFAULT_JAVA_MAIN_CLASS = "pruebas.Prueba1"
DEFAULT_DIRECT_COMMAND = "run-all"
REQUIRED_SETTINGS_FILES = (
    "simulation_parameters.properties",
    "applications.xml",
    "edge_devices.xml",
    "edge_datacenters.xml",
    "cloud.xml",
)

OFFLINE_ALGORITHMS = {
    "RANDOM",
    "RANDOM_GOOD",
    "LOCAL",
    "CLOSEST",
    "MIST",
    "EDGE",
    "CLOUD",
    "ROUND_ROBIN",
    "TRADE_OFF",
    "INCREASE_LIFETIME",
    "LATENCY_ENERGY_AWARE",
    "WEIGHT_GREEDY",
    "TEST",
    "RL",
    "RL_MULTILAYER",
    "RL_MULTILAYER_DISABLED",
    "RL_MULTILAYER_EMPTY",
    "FUZZY_LOGIC",
    "MAPPO",
    "PPO_NEW",
}

INTERACTIVE_ALGORITHMS = {
    "PPO",
}


@dataclass
class SettingsSummary:
    settings_dir: str
    settings_name: str
    algorithms: List[str]
    architectures: List[str]
    device_counts: List[int]
    total_scenarios: int
    algorithm_modes: Dict[str, str]
    required_files: List[str]
    properties: Dict[str, str]

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


@dataclass
class RunResult:
    mode: str
    settings_dir: str
    settings_snapshot_dir: str
    output_root: str
    run_root: str
    simulation_output_dir: str
    log_path: str
    manifest_path: str
    command: List[str]
    summary: Dict[str, object]
    returncode: int
    started_at: str
    finished_at: str

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


def inspect_settings(settings_dir: Path | str) -> SettingsSummary:
    resolved_dir = _resolve_existing_dir(settings_dir)
    _validate_settings_dir(resolved_dir)
    properties = _read_properties(resolved_dir / "simulation_parameters.properties")

    algorithms = _split_csv(properties, "orchestration_algorithms")
    architectures = _split_csv(properties, "orchestration_architectures")
    device_counts = _build_device_counts(properties)

    if not algorithms:
        raise ValueError("No orchestration_algorithms found in simulation_parameters.properties.")
    if not architectures:
        raise ValueError("No orchestration_architectures found in simulation_parameters.properties.")
    if not device_counts:
        raise ValueError("No device counts could be derived from the current settings.")

    algorithm_modes = {algorithm: algorithm_mode(algorithm) for algorithm in algorithms}
    total_scenarios = len(algorithms) * len(architectures) * len(device_counts)

    return SettingsSummary(
        settings_dir=str(resolved_dir),
        settings_name=resolved_dir.name,
        algorithms=algorithms,
        architectures=architectures,
        device_counts=device_counts,
        total_scenarios=total_scenarios,
        algorithm_modes=algorithm_modes,
        required_files=list(REQUIRED_SETTINGS_FILES),
        properties=properties,
    )


def run_settings(
    settings_dir: Path | str,
    output_root: Path | str | None = None,
    mode: str = "all",
    compile_first: bool = True,
    java_main_class: str = DEFAULT_JAVA_MAIN_CLASS,
    ) -> RunResult:
    normalized_mode = _normalize_mode(mode)
    summary = inspect_settings(settings_dir)
    _validate_summary_for_run(summary, normalized_mode)

    resolved_settings_dir = Path(summary.settings_dir)
    resolved_output_root = _resolve_output_root(output_root)
    run_root = _create_run_root(resolved_output_root, summary.settings_name)
    settings_snapshot_dir = run_root / "settings_snapshot"
    simulation_output_dir = run_root / "simulation_output"
    log_path = run_root / "run.log"
    manifest_path = run_root / "manifest.json"

    shutil.copytree(resolved_settings_dir, settings_snapshot_dir)
    simulation_output_dir.mkdir(parents=True, exist_ok=True)

    manifest = {
        "status": "running",
        "mode": normalized_mode,
        "settings_dir": summary.settings_dir,
        "settings_snapshot_dir": str(settings_snapshot_dir),
        "output_root": str(resolved_output_root),
        "run_root": str(run_root),
        "simulation_output_dir": str(simulation_output_dir),
        "log_path": str(log_path),
        "java_main_class": java_main_class,
        "summary": summary.to_dict(),
        "started_at": _timestamp_iso(),
    }
    _write_manifest(manifest_path, manifest)

    command = [
        _resolve_maven_executable(),
        "-q",
        "-DskipTests",
        f"-DsettingsPath={_with_trailing_separator(settings_snapshot_dir)}",
        f"-DoutputPath={_with_trailing_separator(simulation_output_dir)}",
        f"-Dexec.mainClass={java_main_class}",
        "exec:java",
    ]

    returncode = -1
    finished_at = _timestamp_iso()
    with log_path.open("a", encoding="utf-8", buffering=1) as log_file:
        try:
            _log_line(log_file, f"mode={normalized_mode}")
            _log_line(log_file, f"settings_dir={resolved_settings_dir}")
            _log_line(log_file, f"settings_snapshot_dir={settings_snapshot_dir}")
            _log_line(log_file, f"simulation_output_dir={simulation_output_dir}")
            _log_line(log_file, f"java_main_class={java_main_class}")
            _log_line(log_file, f"summary={json.dumps(summary.to_dict(), ensure_ascii=True)}")

            if compile_first:
                compile_command = [_resolve_maven_executable(), "-q", "-DskipTests", "compile"]
                _log_line(log_file, f"compile_command={' '.join(compile_command)}")
                _run_command(compile_command, log_file)

            _log_line(log_file, f"run_command={' '.join(command)}")
            returncode = _run_command(
                command,
                log_file,
                failure_markers=("Main- The simulation has been terminated due to an unexpected error",),
            )
            finished_at = _timestamp_iso()

            manifest["status"] = "completed"
            manifest["returncode"] = returncode
            manifest["command"] = command
            manifest["finished_at"] = finished_at
            _write_manifest(manifest_path, manifest)
        except Exception as exc:
            finished_at = _timestamp_iso()
            manifest["status"] = "failed"
            manifest["returncode"] = returncode
            manifest["command"] = command
            manifest["finished_at"] = finished_at
            manifest["error"] = str(exc)
            _write_manifest(manifest_path, manifest)
            raise

    return RunResult(
        mode=normalized_mode,
        settings_dir=summary.settings_dir,
        settings_snapshot_dir=str(settings_snapshot_dir),
        output_root=str(resolved_output_root),
        run_root=str(run_root),
        simulation_output_dir=str(simulation_output_dir),
        log_path=str(log_path),
        manifest_path=str(manifest_path),
        command=command,
        summary=summary.to_dict(),
        returncode=returncode,
        started_at=manifest["started_at"],
        finished_at=finished_at,
    )


def inspect_default_settings() -> SettingsSummary:
    return inspect_settings(DEFAULT_SETTINGS_DIR)


def run_default_settings() -> RunResult:
    mode = "one" if DEFAULT_DIRECT_COMMAND == "run-one" else "all"
    return run_settings(
        settings_dir=DEFAULT_SETTINGS_DIR,
        output_root=DEFAULT_OUTPUT_ROOT,
        mode=mode,
        compile_first=True,
        java_main_class=DEFAULT_JAVA_MAIN_CLASS,
    )


def algorithm_mode(algorithm: str) -> str:
    normalized = algorithm.strip().upper()
    if normalized in OFFLINE_ALGORITHMS:
        return "offline"
    if normalized in INTERACTIVE_ALGORITHMS:
        return "interactive"
    return "unknown"


def _normalize_mode(mode: str) -> str:
    normalized = mode.strip().lower()
    if normalized in {"all", "run-all"}:
        return "all"
    if normalized in {"one", "single", "run-one"}:
        return "one"
    raise ValueError(f"Unsupported mode '{mode}'. Expected 'all' or 'one'.")


def _validate_summary_for_run(summary: SettingsSummary, mode: str) -> None:
    interactive = [name for name, value in summary.algorithm_modes.items() if value == "interactive"]
    unknown = [name for name, value in summary.algorithm_modes.items() if value == "unknown"]

    if interactive:
        raise ValueError(
            "The selected settings include interactive algorithms that require an env server/client flow and "
            f"are not wired into run_simulation.py yet: {', '.join(interactive)}"
        )
    if unknown:
        raise ValueError(
            "The selected settings include algorithms that are not recognized by run_simulation.py: "
            f"{', '.join(unknown)}"
        )
    if mode == "one" and summary.total_scenarios != 1:
        raise ValueError(
            f"run-one requires exactly 1 scenario, but this settings directory resolves to "
            f"{summary.total_scenarios}. Use run-all instead."
        )
    _validate_java_compatible_properties(summary.properties)


def _resolve_existing_dir(path_value: Path | str) -> Path:
    raw_path = Path(path_value).expanduser()
    candidates = [raw_path]
    if not raw_path.is_absolute():
        candidates.append(Path.cwd() / raw_path)
        candidates.append(REPO_ROOT / raw_path)

    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError(f"Settings directory not found: {path_value}")


def _resolve_output_root(path_value: Path | str | None) -> Path:
    if path_value is None:
        return DEFAULT_OUTPUT_ROOT.resolve()

    raw_path = Path(path_value).expanduser()
    if raw_path.is_absolute():
        return raw_path.resolve()

    cwd_candidate = (Path.cwd() / raw_path).resolve()
    repo_candidate = (REPO_ROOT / raw_path).resolve()
    if cwd_candidate.exists():
        return cwd_candidate
    if repo_candidate.exists():
        return repo_candidate
    return cwd_candidate


def _validate_settings_dir(settings_dir: Path) -> None:
    if not settings_dir.is_dir():
        raise NotADirectoryError(f"Settings path is not a directory: {settings_dir}")
    missing = [name for name in REQUIRED_SETTINGS_FILES if not (settings_dir / name).is_file()]
    if missing:
        raise FileNotFoundError(
            f"Settings directory is incomplete: {settings_dir}. Missing files: {', '.join(missing)}"
        )


def _read_properties(path: Path) -> Dict[str, str]:
    properties: Dict[str, str] = {}
    for raw_line in _read_text_with_fallback(path).splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#") or stripped.startswith("!"):
            continue
        separator_index = _find_property_separator(stripped)
        if separator_index == -1:
            continue
        key = stripped[:separator_index].strip()
        value = stripped[separator_index + 1 :].strip()
        if key:
            properties[key] = value
    return properties


def _find_property_separator(line: str) -> int:
    for separator in ("=", ":"):
        index = line.find(separator)
        if index != -1:
            return index
    return -1


def _read_text_with_fallback(path: Path) -> str:
    encodings = ("utf-8", "utf-8-sig", "cp1252", "latin-1")
    last_error: Optional[UnicodeDecodeError] = None
    for encoding in encodings:
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError as exc:
            last_error = exc
    if last_error is not None:
        raise last_error
    raise RuntimeError(f"Unable to read text file: {path}")


def _split_csv(properties: Dict[str, str], key: str) -> List[str]:
    raw_value = properties.get(key, "")
    return [item.strip().upper() for item in raw_value.split(",") if item.strip()]


def _build_device_counts(properties: Dict[str, str]) -> List[int]:
    try:
        minimum = int(properties["min_number_of_edge_devices"].strip())
        maximum = int(properties["max_number_of_edge_devices"].strip())
        step = int(properties["edge_device_counter_size"].strip())
    except KeyError as exc:
        raise KeyError(f"Missing required property: {exc.args[0]}") from exc
    except ValueError as exc:
        raise ValueError("Device count properties must be integers.") from exc

    if step <= 0:
        raise ValueError("edge_device_counter_size must be greater than 0.")
    if minimum > maximum:
        raise ValueError("min_number_of_edge_devices cannot be greater than max_number_of_edge_devices.")

    return list(range(minimum, maximum + 1, step))


def _validate_java_compatible_properties(properties: Dict[str, str]) -> None:
    seed_value = properties.get("random_seed")
    if seed_value is None:
        return
    normalized = seed_value.strip()
    if not normalized:
        return
    try:
        int(normalized)
    except ValueError as exc:
        raise ValueError(
            "simulation_parameters.properties contains an invalid random_seed value for the current Java parser: "
            f"{seed_value!r}. Use an integer or leave the property empty."
        ) from exc


def _create_run_root(output_root: Path, settings_name: str) -> Path:
    runs_root = output_root / "runs"
    runs_root.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    slug = _slugify(settings_name)
    base_name = f"{timestamp}_{slug}"
    candidate = runs_root / base_name
    suffix = 2
    while candidate.exists():
        candidate = runs_root / f"{base_name}_{suffix}"
        suffix += 1

    candidate.mkdir(parents=True, exist_ok=False)
    return candidate


def _resolve_maven_executable() -> str:
    for candidate in ("mvn.cmd", "mvn.bat", "mvn"):
        resolved = shutil.which(candidate)
        if resolved:
            return resolved
    raise FileNotFoundError("Unable to locate Maven executable. Expected one of: mvn.cmd, mvn.bat, mvn")


def _slugify(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9._-]+", "_", value.strip())
    return slug.strip("._-") or "settings"


def _write_manifest(path: Path, payload: Dict[str, object]) -> None:
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")


def _run_command(
    command: Sequence[str],
    log_file,
    failure_markers: Sequence[str] = (),
) -> int:
    process = subprocess.Popen(
        list(command),
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert process.stdout is not None
    marker_hits: List[str] = []
    try:
        for line in process.stdout:
            text = line.rstrip("\n")
            print(text)
            log_file.write(text + "\n")
            for marker in failure_markers:
                if marker in text:
                    marker_hits.append(marker)
    finally:
        process.stdout.close()

    returncode = process.wait()
    if returncode != 0:
        raise RuntimeError(f"Command failed with exit code {returncode}: {' '.join(command)}")
    if marker_hits:
        unique_markers = ", ".join(sorted(set(marker_hits)))
        raise RuntimeError(
            f"Command completed with exit code 0 but reported a simulation failure ({unique_markers}): "
            f"{' '.join(command)}"
        )
    return returncode


def _with_trailing_separator(path: Path) -> str:
    return path.resolve().as_posix().rstrip("/") + "/"


def _timestamp_iso() -> str:
    return datetime.now().isoformat(timespec="seconds")


def _log_line(log_file, message: str) -> None:
    timestamp = _timestamp_iso()
    line = f"{timestamp} {message}"
    print(line)
    log_file.write(line + "\n")


def _format_summary(summary: SettingsSummary) -> str:
    device_counts = ", ".join(str(item) for item in summary.device_counts)
    algorithms = ", ".join(summary.algorithms)
    architectures = ", ".join(summary.architectures)
    mode_lines = [
        f"  - {algorithm}: {summary.algorithm_modes[algorithm]}"
        for algorithm in summary.algorithms
    ]
    return "\n".join(
        [
            f"Settings directory: {summary.settings_dir}",
            f"Settings name: {summary.settings_name}",
            f"Algorithms: {algorithms}",
            f"Architectures: {architectures}",
            f"Device counts: {device_counts}",
            f"Total scenarios: {summary.total_scenarios}",
            "Algorithm modes:",
            *mode_lines,
        ]
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect and run PureEdgeSim settings-driven simulations."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    inspect_parser = subparsers.add_parser("inspect", help="Inspect a settings directory.")
    inspect_parser.add_argument(
        "--settings-dir",
        default=str(DEFAULT_SETTINGS_DIR),
        help="Path to the settings directory.",
    )
    inspect_parser.add_argument("--json", action="store_true", help="Print the summary as JSON.")

    for command_name in ("run-one", "run-all"):
        run_parser = subparsers.add_parser(command_name, help=f"Execute {command_name}.")
        run_parser.add_argument(
            "--settings-dir",
            default=str(DEFAULT_SETTINGS_DIR),
            help="Path to the settings directory.",
        )
        run_parser.add_argument(
            "--output-root",
            default=str(DEFAULT_OUTPUT_ROOT),
            help="Root directory where run artifacts will be stored.",
        )
        run_parser.add_argument(
            "--java-main-class",
            default=DEFAULT_JAVA_MAIN_CLASS,
            help="Java main class used by mvn exec:java.",
        )
        run_parser.add_argument(
            "--skip-compile",
            action="store_true",
            help="Skip 'mvn compile' before launching the simulation.",
        )

    return parser


def _main(argv: Optional[Sequence[str]] = None) -> int:
    if argv is None and len(sys.argv) == 1:
        summary = inspect_default_settings()
        print(_format_summary(summary))
        result = run_default_settings()
        print(json.dumps(result.to_dict(), indent=2, ensure_ascii=True))
        return 0

    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.command == "inspect":
        summary = inspect_settings(args.settings_dir)
        if args.json:
            print(json.dumps(summary.to_dict(), indent=2, ensure_ascii=True))
        else:
            print(_format_summary(summary))
        return 0

    mode = "one" if args.command == "run-one" else "all"
    result = run_settings(
        settings_dir=args.settings_dir,
        output_root=args.output_root,
        mode=mode,
        compile_first=not args.skip_compile,
        java_main_class=args.java_main_class,
    )
    print(json.dumps(result.to_dict(), indent=2, ensure_ascii=True))
    return 0


def main() -> None:
    try:
        raise SystemExit(_main())
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
