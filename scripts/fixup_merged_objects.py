"""objcopy pass for the merged test binary (see generate_merged_tests.py).

After the wrapper objects compile, each one's app_main is redefined to its
unique suite entry name, defined Unity hooks are redefined the same way, and
every other strong global symbol is localized so same-named symbols from
different suites cannot collide in the merged link.

Idempotent: safe to run again after a rebuild or a partial earlier pass.

Usage: python3 scripts/fixup_merged_objects.py [project_dir]
"""

import json
import os
import subprocess
import sys

STRONG_TYPES = "TDBR"


def tool_paths():
    pio_home = os.environ.get("PIO_HOME", os.path.expanduser("~/.platformio"))
    bin_dir = os.path.join(
        pio_home, "packages", "toolchain-xtensa-esp-elf", "bin"
    )
    return (
        os.path.join(bin_dir, "xtensa-esp-elf-nm"),
        os.path.join(bin_dir, "xtensa-esp-elf-objcopy"),
    )


def strong_globals(nm, obj):
    """Global defined symbols of an object (types T, D, B or R)."""
    symbols = {}
    for line in subprocess.check_output([nm, obj], text=True).splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) == 3 and parts[1] in STRONG_TYPES:
            symbols[parts[2]] = parts[1]
    return symbols


def fixup(nm, objcopy, obj, slug, hooks):
    if not os.path.exists(obj):
        print("skip (not built yet):", os.path.basename(obj))
        return

    symbols = strong_globals(nm, obj)
    run_name = "suite_%s_run" % slug
    final_names = {run_name}

    cmd = [objcopy]
    if "app_main" in symbols:
        cmd += ["--redefine-sym", "app_main=" + run_name]
    elif symbols.get(run_name) == "t":
        # A previous partial pass localized it; make it global again.
        cmd += ["--globalize-symbol", run_name]

    for hook in hooks:
        hook_name = "suite_%s_%s" % (slug, hook)
        final_names.add(hook_name)
        if hook in symbols:
            cmd += ["--redefine-sym", "%s=%s" % (hook, hook_name)]
        elif symbols.get(hook_name) == "t":
            cmd += ["--globalize-symbol", hook_name]

    for symbol in sorted(set(symbols) - {"app_main"} - final_names):
        cmd += ["--localize-symbol", symbol]

    if len(cmd) > 1:
        subprocess.run(cmd + [obj], check=True)
        print("fixed", os.path.basename(obj))
    else:
        print("already fixed", os.path.basename(obj))


def main():
    project_dir = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else os.getcwd())
    manifest_path = os.path.join(
        project_dir, "test", "test_all", "merged_manifest.json"
    )
    if not os.path.exists(manifest_path):
        raise SystemExit(
            "error: %s not found - build the test_coverage environment once to "
            "generate it" % manifest_path
        )

    nm, objcopy = tool_paths()
    with open(manifest_path) as manifest:
        suites = json.load(manifest)["suites"]

    for suite in suites:
        obj = os.path.join(project_dir, suite["object"])
        fixup(nm, objcopy, obj, suite["slug"], suite["hooks"])


if __name__ == "__main__":
    main()
