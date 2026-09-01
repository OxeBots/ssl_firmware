"""PlatformIO extra script: marks coverage builds for the root CMakeLists.txt."""

import os

_Import = globals().get("Import")
if _Import is not None:
    _Import("env")
    env = globals()["env"]
    # The root CMakeLists.txt compiles project sources with --coverage when
    # this environment variable is set. Only the test_coverage environment
    # sets it, so regular builds stay untouched.
    if env["PIOENV"] == "test_coverage":
        os.environ["COVERAGE_BUILD"] = "1"
