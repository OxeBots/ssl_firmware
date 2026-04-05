"""PlatformIO extra script: compiles .bitproto files and generates C code."""

import glob
import os
import urllib.request
from os.path import basename, exists, getmtime, join, splitext


def ensure_bitproto_runtime(env, output_dir):
    """
    Download bitproto.h and bitproto.c from the official GitHub repository
    at the specified version, but only if they are not already present.
    Version can be set in platformio.ini as bitproto_version (e.g., v1.2.2 or 1.2.2).
    Defaults to v1.2.2.
    """
    version = env.GetProjectOption("bitproto_version", "v1.2.2")
    # Ensure version has 'v' prefix (tag format in repo)
    if not version.startswith("v"):
        version = "v" + version

    base_url = f"https://raw.githubusercontent.com/hit9/bitproto/{version}/lib/c"
    files = ["bitproto.h", "bitproto.c"]

    # Check if both files already exist
    all_exist = True
    for filename in files:
        dst = join(output_dir, filename)
        if not exists(dst):
            all_exist = False
            break

    if all_exist:
        print("bitproto runtime files already present. Skipping download.")
        return

    # Download missing files (or all if some missing)
    for filename in files:
        dst = join(output_dir, filename)
        url = f"{base_url}/{filename}"
        print(f"...Downloading {filename} version {version} from GitHub...")
        try:
            with urllib.request.urlopen(url) as response:
                if response.getcode() != 200:
                    raise Exception(f"HTTP {response.getcode()}")
                content = response.read()
            with open(dst, "wb") as f:
                f.write(content)
            print(f"Saved {filename} to {dst}")
        except Exception as e:
            print(f"X Failed to download {filename}: {e}")
            env.Exit(1)


def compile_bitprotos(env):
    """
    Compile all .bitproto files using the 'bitproto' tool.
    Generated files are placed in the build directory, headers are added to CPPPATH,
    and generated C sources (including the runtime bitproto.c) are built into a
    static library that is linked automatically.
    """
    # Configuration
    proto_dir = env.GetProjectOption("proto_dir", "proto")
    proto_dir = env.subst(proto_dir)

    lang = "c"
    build_dir = env.subst("$BUILD_DIR")
    output_dir = join(build_dir, "proto_gen")
    bitproto_cmd = "bitproto {lang} {input} {out_dir}"

    if not exists(proto_dir):
        print(f"  Proto directory '{proto_dir}' not found. Skipping bitproto compilation.")
        return

    proto_files = glob.glob(join(proto_dir, "*.bitproto"))
    if not proto_files:
        print(f"  No .bitproto files found in '{proto_dir}'. Nothing to do.")
        return

    os.makedirs(output_dir, exist_ok=True)

    # Determine which files need (re)generation
    files_to_generate = []
    for pf in proto_files:
        base = splitext(basename(pf))[0]
        out_h = join(output_dir, base + ".h")
        out_c = join(output_dir, base + ".c")
        if (
            (not exists(out_h))
            or (not exists(out_c))
            or (getmtime(pf) > getmtime(out_h))
            or (getmtime(pf) > getmtime(out_c))
        ):
            files_to_generate.append(pf)

    if not files_to_generate:
        print("All .bitproto files are up to date.")
    else:
        print(f"Compiling {len(files_to_generate)} .bitproto file(s) with bitproto...")
        for pf in files_to_generate:
            cmd = bitproto_cmd.format(lang=lang, out_dir=output_dir, input=pf)
            print(f"   Running: {cmd}")
            result = env.Execute(cmd)
            if result != 0:
                print(f"X Failed to compile {pf}")
                env.Exit(1)
        print("Bitproto compilation finished.")

    # Ensure bitproto runtime (header and implementation) is present
    ensure_bitproto_runtime(env, output_dir)

    # Add generated headers to the compiler's include path
    env.Append(CPPPATH=[output_dir])

    # Build static library from all C sources in output_dir
    # This includes generated _bp.c files AND the runtime bitproto.c
    all_c_sources = glob.glob(join(output_dir, "*.c"))
    if all_c_sources:
        lib = env.StaticLibrary(target=join(output_dir, "libbitproto_generated"), source=all_c_sources)
        print(f"Library target: {lib[0].abspath}")
        env.Append(LIBS=[lib])
    else:
        print("No C source files found in output directory.")


try:
    Import("env")
    compile_bitprotos(env)
except ImportError:
    print("This script is intended to be run inside PlatformIO.")
    exit(1)
