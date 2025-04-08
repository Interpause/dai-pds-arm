#!/usr/bin/env bash
# This script calculates a checksum for all code files to determine if re-upload
# is needed. That is because pio upload still takes about 9 seconds to figure out
# if it needs to rebuild then upload.

set -eo pipefail

# Directory this script is in.
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Can't filter for symlinks that are files so use test -f instead.
find $SCRIPT_DIR/../include $SCRIPT_DIR/../lib $SCRIPT_DIR/../src \
  \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) \
  -exec test -f {} \; -exec cat {} + | cat - $SCRIPT_DIR/../platformio.ini | cksum | awk '{print $1}'
