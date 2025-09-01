import json
from os.path import join as pjoin

def update_flylint_include_config(project_dir):
    """
    Reads include paths from .vscode/c_cpp_properties.json and updates
    .vscode/settings.json for the c-cpp-flylint extension.

    Args:
        project_dir (str): The root directory of the PlatformIO project.
    """
    c_cpp_properties_path = pjoin(project_dir, ".vscode", "c_cpp_properties.json")
    settings_path = pjoin(project_dir, ".vscode", "settings.json")

    try:
        with open(c_cpp_properties_path, 'r') as f:
            lines = f.readlines()

        # Filter out comment lines
        json_lines = [line for line in lines if not line.strip().startswith("//")]

        # Join the remaining lines to form a valid JSON string
        json_content = "".join(json_lines)

        try:
            c_cpp_config = json.loads(json_content)
        except json.JSONDecodeError as e:
            print(f"Error: Could not decode JSON from: {c_cpp_properties_path} after removing comments. Error details: {e}")
            return

    except FileNotFoundError:
        print(f"Error: File not found: {c_cpp_properties_path}")
        return

    include_paths = []
    for config in c_cpp_config.get("configurations", []):
        if config.get("name") == "PlatformIO":
            include_paths.extend(config.get("includePath", []))
            break

    include_paths = [path for path in include_paths if path]

    try:
        with open(settings_path, 'r+') as f:
            try:
                settings_config = json.load(f)
            except json.JSONDecodeError:
                settings_config = {}

            # Remove duplicates
            settings_config["c-cpp-flylint.clang.includePaths"] = list(set(include_paths))

            f.seek(0)
            json.dump(settings_config, f, indent=4)
            f.truncate()
        print(f"Successfully updated {settings_path}")

    except FileNotFoundError:
        print(f"Warning: File not found: {settings_path}. Creating it.")
        settings_config = {"c-cpp-flylint.clang.includePaths": list(set(include_paths))}

        with open(settings_path, 'w') as f:
            json.dump(settings_config, f, indent=4)

        print(f"Successfully created and updated {settings_path}")

    except Exception as e:
        print(f"An error occurred while updating {settings_path}: {e}")

try:
    Import("env")
    project_dir = env["PROJECT_DIR"]
    update_flylint_include_config(project_dir)
except ImportError:
    print("Warning: Not running within PlatformIO environment.")
