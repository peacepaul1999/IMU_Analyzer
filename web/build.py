import shutil
import subprocess
import sys

from pathlib import Path


ROOT = Path(__file__).resolve().parent

DIST_DIR = ROOT / "dist"

DATA_DIR = ROOT.parent / "data"

GENERATED_DATA_ENTRIES = (
    "api",
    "charts",
    "config",
    "dom",
    "imu",
    "main.js",
    "ota",
    "types",
    "ui",
    "utils",
    "vendor",
    "websocket",
)


def npm_command() -> str:

    return "npm.cmd" if sys.platform.startswith("win") else "npm"


def run_build() -> None:

    print("[BUILD] TypeScript")

    subprocess.run(
        [npm_command(), "run", "build"],
        cwd=ROOT,
        check=True
    )


def prepare_data_dir() -> None:

    DATA_DIR.mkdir(
        exist_ok=True
    )

    for entry_name in GENERATED_DATA_ENTRIES:

        destination = DATA_DIR / entry_name

        if destination.is_dir():

            shutil.rmtree(
                destination,
                ignore_errors=True
            )

            continue

        if destination.exists():

            destination.unlink()


def copy_build_files() -> None:

    print("[COPY] dist -> data")

    for entry in DIST_DIR.iterdir():

        destination = DATA_DIR / entry.name

        if entry.is_dir():

            shutil.copytree(entry, destination, dirs_exist_ok=True)

            continue

        shutil.copy2(entry, destination)


def main() -> None:

    run_build()

    prepare_data_dir()

    copy_build_files()

    print("[DONE] TS build complete")


if __name__ == "__main__":

    main()
