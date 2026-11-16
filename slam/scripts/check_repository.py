"""Check tracked/source candidates for merge debris, generated files and large binaries."""

import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MARKER = re.compile(r"^(?:<{7} |>{7} |={7}$)", re.MULTILINE)
GENERATED = {"__pycache__", "build", "install", "log", "station_data"}


def check():
    names = (
        subprocess.check_output(
            ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"], cwd=ROOT
        )
        .decode()
        .split("\0")
    )
    failures = []
    for name in sorted(set(filter(None, names))):
        path = ROOT / name
        # A deleted tracked artifact is already being removed by this branch.
        if not path.exists():
            continue
        if any(part in GENERATED for part in Path(name).parts) or path.suffix == ".pyc":
            failures.append(f"Generated/private file in source tree: {name}")
            continue
        if path.is_symlink():
            failures.append(f"Unreviewed symlink in source tree: {name}")
            continue
        if path.stat().st_size > 2_000_000:
            failures.append(f"Large artifact should not be committed: {name}")
        if (
            path.suffix in {".py", ".md", ".toml", ".txt", ".yml", ".yaml", ".sh"}
            or name == ".gitignore"
        ):
            if MARKER.search(path.read_text()):
                failures.append(f"Unresolved merge marker: {name}")
    return failures


if __name__ == "__main__":
    errors = check()
    if errors:
        raise SystemExit("\n".join(errors))
    print("Repository source checks passed.")
