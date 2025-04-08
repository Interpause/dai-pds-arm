#!/usr/bin/env bash
# This script builds and uploads the code to the microprocessor.

set -eo pipefail

# Directory this script is in.
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Check if platformio udev rules present.
if [ ! -f /etc/udev/rules.d/99-platformio-udev.rules ]; then
    echo "udev rules missing. Installing..."
    sudo wget -qO /etc/udev/rules.d/99-platformio-udev.rules https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules
    echo "It is recommended to restart to ensure udev rules take effect."
    exit 1
fi

# Skip uploading if the files are exactly the same.
src_cksum=$($SCRIPT_DIR/checksum.sh)
if echo "$@" | grep -q -- "--skip-same" && [ -f "$SCRIPT_DIR/.upload-checksum" ]; then
    echo "Checking if reupload is needed..."
    old_cksum=$(cat $SCRIPT_DIR/.upload-checksum)
    if [ "$src_cksum" = "$old_cksum" ]; then
        echo "No changes detected. Not reuploading."
        exit 0
    fi
fi

# According to https://docs.platformio.org/en/latest/librarymanager/dependencies.html#installing-dependencies
# pio auto-installs dependencies on build. Wonder what happens if its offline (it uses cached packages even if I delete the .pio folder...)
# pio also tries to auto-detect the correct board. Nice.
# pio also keeps its own checksum for the build cache, so I think spamming upload
# in the ROS node should be fine now.
pio run --verbose -t upload -d $SCRIPT_DIR/..
echo $src_cksum > $SCRIPT_DIR/.upload-checksum
