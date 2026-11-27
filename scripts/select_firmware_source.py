"""Select the ESP-IDF application component for this PlatformIO environment."""

from os.path import join

Import("env")
source_dir = join(env.subst("$PROJECT_DIR"), env.GetProjectOption("custom_firmware_dir"))
env.Replace(PROJECT_SRC_DIR=source_dir)
