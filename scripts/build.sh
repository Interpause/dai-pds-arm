#!/usr/bin/env bash
# Convenience script to rosdep install everything and colcon build everything.

set -eo pipefail

# Directory this script is in.
SCRIPT_DIR=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
# Directory of workspace root, assuming script is in `scripts/`.
WS_DIR=$(realpath "$SCRIPT_DIR"/..)
# Base colcon build command and options.
COLCON_BUILD="colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --symlink-install --continue-on-error"

function usage() {
  echo "Usage: $0 [--full] [--thirdparty] [--src] [<package1> <package2> ...]"
  echo "Options:"
  echo "  -h, --help: Show this help message."
  echo "  -v, --verbose: Enable verbose output."
  echo "  --full: Perform full build of all packages."
  echo "  --thirdparty: Build only packages in thirdparty/."
  echo "  --src: Build only user source packages in src/."
  echo "The default is selective build of specified packages."
  echo "Specifying a build set means any packages listed will be ignored from build instead."
}

# Normalize options.
OPTS=$(getopt -o hv -l help,verbose,full,thirdparty,src -n "$0" -- "$@")
if [ $? -ne 0 ]; then usage; exit 1; fi
eval set -- "$OPTS"

function run_clean_shell() {
  # Run commands in login shell to avoid underlay override warning.
  if [ "$VERBOSE" = true ]; then echo "Running: $@" >&2; fi
  env -iC "$WS_DIR" bash -c "$(echo ". /opt/ros/humble/setup.bash; $@")"
}

function build_selective() {
  # Build in login shell to avoid underlay override warning.
  # PYTHONWARNINGS below: Suppress warnings due to https://github.com/colcon/colcon-core/issues/454.
  if [ "$VERBOSE" = true ]; then echo "Packages: $@" >&2; fi
  if [ -d "$WS_DIR"/install/ ]; then
    run_clean_shell "cd install/ && rm -rf $@"
  fi
  run_clean_shell "\
    PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources\
    $COLCON_BUILD --packages-select $@\
  "
}

# First arg is the folder, the rest are packages to ignore.
function scan_folder() {
  # Use colcon to find all valid packages.
  if [ "$VERBOSE" = true ]; then echo "Scanning: $1, Ignoring: ${@:2}" >&2; fi
  source /opt/ros/humble/setup.bash
  echo $(colcon list --names-only --base-paths $1 --packages-ignore ${@:2})
}

# Prep for full clean build.
function prep_full() {
  source /opt/ros/humble/setup.bash
  rosdep update
  (cd "$WS_DIR" && {
    rosdep install --from-paths src/ thirdparty/ --ignore-src -y
    pip install -r requirements.txt
    rm -rf install/ build/
  })
}

MODE=""
VERBOSE=false
while true; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    -v|--verbose)
      shift
      VERBOSE=true
      ;;
    --full)
      shift
      if [ "$MODE" ]; then
        echo "$0: ERROR: Can only specify one build set."
        exit 1
      fi
      MODE="full"
      ;;
    --thirdparty)
      shift
      if [ "$MODE" ]; then
        echo "$0: ERROR: Can only specify one build set."
        exit 1
      fi
      MODE="thirdparty"
      ;;
    --src)
      shift
      if [ "$MODE" ]; then
        echo "$0: ERROR: Can only specify one build set."
        exit 1
      fi
      MODE="src"
      ;;
    --)
      shift
      if [ -z "$MODE" ]; then
        MODE="selective"
      fi
      break
      ;;
    *)
      echo "$0: ERROR: Invalid option." 
      exit 1
      ;;
  esac
done
if [ "$VERBOSE" = true ]; then echo "Mode: $MODE" >&2; fi

if [ "$MODE" = "full" ]; then
  PACKAGES=$(cd "$WS_DIR" && {
    scan_folder "src/" $@
    scan_folder "thirdparty/" $@
  })
  prep_full
elif [ "$MODE" = "thirdparty" ]; then
  PACKAGES=$(cd "$WS_DIR" && scan_folder "thirdparty/" $@)
elif [ "$MODE" = "src" ]; then
  PACKAGES=$(cd "$WS_DIR" && scan_folder "src/" $@)
elif [ "$MODE" = "selective" ]; then
  PACKAGES=$@
else
  echo "$0: ERROR: IDK how you managed to get here."
  usage
  exit 1
fi

if [ -z "$PACKAGES" ]; then
  echo "$0: ERROR: No packages or build set specified."
  usage
  exit 1
fi

build_selective $PACKAGES
