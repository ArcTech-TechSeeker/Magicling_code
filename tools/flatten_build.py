Import("env")

from pathlib import Path
import shutil


PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
PLATFORMIO_BUILD_DIR = Path(env.subst("$BUILD_DIR"))
FLAT_BUILD_DIR = PROJECT_DIR / "build"
OUTPUT_NAME = "Magicling_code.ino.bin"


def clean_flat_build_dir() -> None:
    FLAT_BUILD_DIR.mkdir(parents=True, exist_ok=True)

    for item in FLAT_BUILD_DIR.iterdir():
        if item.is_dir():
            shutil.rmtree(item)
        else:
            item.unlink()


def flatten_build(source, target, env):
    clean_flat_build_dir()

    firmware_bin = PLATFORMIO_BUILD_DIR / "firmware.bin"
    if not firmware_bin.exists():
        print(f"[flatten_build] missing: {firmware_bin}")
        return

    shutil.copy2(firmware_bin, FLAT_BUILD_DIR / OUTPUT_NAME)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", flatten_build)