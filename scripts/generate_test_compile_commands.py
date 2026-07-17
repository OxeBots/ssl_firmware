#!/usr/bin/env python3
"""Append test file entries to compile_commands.json.

PlatformIO's compiledb skips test/ files. This clones the src/main.cpp
compile command, adds all lib/ include dirs, and emits one entry per test .cpp.

Dual-mode: CLI (via regenerate_compiledb.sh) or PlatformIO extra script
(post: in platformio.ini) wired to the compiledb target via SCons.
"""

import json
import os
import re
import shutil
import sys
from glob import glob

# Bake absolute xtensa driver paths so clangd never falls back to bundled clang
# + host headers (cannot parse ESP-IDF/vt-linalg).
_BARE_DRIVERS = ("xtensa-esp32-elf-g++", "xtensa-esp32-elf-gcc")


def absolutize_drivers(entries):
    """Rewrite bare xtensa driver names to absolute toolchain paths (best-effort)."""
    resolved = {name: shutil.which(name) for name in _BARE_DRIVERS}
    changed = 0
    for entry in entries:
        cmd = entry.get("command", "")
        if not cmd:
            continue
        for name in _BARE_DRIVERS:
            abs_path = resolved[name]
            if abs_path is None:
                continue
            # The driver is the first whitespace-delimited token of the command.
            if cmd.startswith(name + " "):
                entry["command"] = abs_path + cmd[len(name) :]
                changed += 1
                break
    return changed


def append_test_entries(workspace):
    """Append one compile_commands.json entry per test .cpp file.

    Idempotent: stale test/ entries are removed first, safe to run repeatedly.

    Returns number of entries added, or -1 on error.
    """
    cc_path = os.path.join(workspace, "compile_commands.json")

    if not os.path.exists(cc_path):
        print("compile_commands.json not found — run 'pio run -t compiledb' first.", file=sys.stderr)
        return -1

    with open(cc_path) as f:
        entries = json.load(f)

    # Bake absolute xtensa driver paths into every entry.
    absolutize_drivers(entries)

    # Find a C++ template entry (prefer src/main.cpp).
    template = None
    for entry in entries:
        if entry["file"] == "src/main.cpp":
            template = entry
            break
    if template is None:
        for entry in entries:
            if entry["file"].endswith(".cpp") and not entry["file"].startswith("/"):
                template = entry
                break
    if template is None:
        print("No C++ compile command found to use as template.", file=sys.stderr)
        return -1

    cmd = template["command"]

    # Collect every lib/ subdirectory (relative paths match existing -I flags).
    lib_dirs = sorted(
        os.path.relpath(d.rstrip("/"), workspace) for d in glob(os.path.join(workspace, "lib/*/")) if os.path.isdir(d)
    )

    # Build extra -I flags for missing lib dirs.
    existing_includes = set(re.findall(r"-I(\S+)", cmd))
    extra_includes = [
        d for d in lib_dirs if d not in existing_includes and os.path.join(workspace, d) not in existing_includes
    ]
    extra_i_flags = " ".join(f"-I{d}" for d in extra_includes)

    # Insert extra -I flags after the last existing -I flag.
    last_i = cmd.rfind(" -I")
    if last_i != -1:
        # Find the end of the last -I... token
        rest = cmd[last_i + 3 :]
        match = re.match(r"(\S+)", rest)
        if match:
            insert_pos = last_i + 3 + match.end()
            new_cmd_template = cmd[:insert_pos] + " " + extra_i_flags + cmd[insert_pos:]
        else:
            new_cmd_template = cmd + " " + extra_i_flags
    else:
        new_cmd_template = cmd + " " + extra_i_flags

    # Discover all test .cpp files (workspace-relative paths).
    test_files = sorted(glob(os.path.join(workspace, "test/**/*.cpp"), recursive=True))
    if not test_files:
        print("No test files found.")
        return 0

    # Remove stale test/ entries.
    entries = [e for e in entries if not e["file"].startswith("test/")]

    added = 0
    template_file = template["file"]
    for test_file in test_files:
        rel_test_file = os.path.relpath(test_file, workspace)
        # Replace template file path with test file path in the command.
        parts = new_cmd_template.rsplit(template_file, 1)
        if len(parts) == 2:
            test_cmd = parts[0] + rel_test_file + parts[1]
        else:
            test_cmd = new_cmd_template + " " + rel_test_file

        entries.append(
            {
                "command": test_cmd,
                "directory": workspace,
                "file": rel_test_file,
            }
        )
        added += 1

    with open(cc_path, "w") as f:
        json.dump(entries, f, indent=4)

    print(f"Added {added} test file entries to compile_commands.json")
    return added


def main():
    """CLI entry point (current working directory as workspace)."""
    added = append_test_entries(os.getcwd())
    return 0 if added >= 0 else 1


_Import = globals().get("Import")

if _Import is not None:
    _Import("env")
    env = globals()["env"]

    def _register_compiledb_postaction(env):
        """Wire into the compiledb target via a Command that sources compile_commands.json.

        env.AddPostAction doesn't work here (compiledb alias created after
        post extra-scripts load). Instead, register a Command target that
        sources the compile_commands.json node and add it to the alias.
        """
        if "compiledb" not in globals().get("COMMAND_LINE_TARGETS", ()):
            return

        def _append_after_compiledb(target, source, env):
            append_test_entries(env.subst("$PROJECT_DIR"))

        cc_node = env.File(env.subst("$COMPILATIONDB_PATH"))
        stamp = env.Command("$BUILD_DIR/append_test_entries.stamp", cc_node, _append_after_compiledb)
        env.Alias("compiledb", stamp)
        env.AlwaysBuild(stamp)

    _register_compiledb_postaction(env)
else:
    if __name__ == "__main__":
        sys.exit(main())
