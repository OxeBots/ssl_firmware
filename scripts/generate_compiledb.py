"""
Auto-generate compile_commands.json only when project config changes.

Checks:
- platformio.ini (build flags, lib_deps, etc.)
- sdkconfig.defaults (ESP-IDF config)
- CMakeLists.txt (build structure)

If none of these changed since last build, skips regeneration.
"""

import hashlib
import os

Import("env")
HASH_FILE = os.path.join(env.subst("$PROJECT_DIR"), ".pio", "compiledb_hash")

KEY_FILES = [
    "$PROJECT_DIR/platformio.ini",
    "$PROJECT_DIR/sdkconfig.defaults",
    "$PROJECT_DIR/CMakeLists.txt",
]


def compute_config_hash():
    """Hash key config files to detect project changes."""
    hasher = hashlib.md5()
    for path in KEY_FILES:
        resolved = env.subst(path)
        if os.path.exists(resolved):
            with open(resolved, "rb") as f:
                hasher.update(f.read())
    return hasher.hexdigest()


def load_stored_hash():
    """Load the hash from the last generation."""
    if os.path.exists(HASH_FILE):
        with open(HASH_FILE, "r") as f:
            return f.read().strip()
    return None


def save_hash(value):
    """Save the current config hash."""
    os.makedirs(os.path.dirname(HASH_FILE), exist_ok=True)
    with open(HASH_FILE, "w") as f:
        f.write(value)


def generate_compiledb_if_needed(*args, **kwargs):
    """Regenerate compile_commands.json only if config changed."""
    current_hash = compute_config_hash()
    stored_hash = load_stored_hash()

    # Always regenerate on first build or if config changed
    if current_hash != stored_hash:
        print("Generating compile_commands.json (config changed)...")
        env.Execute("pio run -t compiledb")
        save_hash(current_hash)
    else:
        print("compile_commands.json is up to date.")


env.AddPostAction("buildprog", generate_compiledb_if_needed)
