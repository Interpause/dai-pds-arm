#!/usr/bin/env bash
# Insert commands to run when container is started for the first time.

set -eo pipefail

# Allow apt index & package cache to persist.
sudo rm -f /etc/apt/apt.conf.d/docker-clean
echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' | sudo tee /etc/apt/apt.conf.d/keep-cache > /dev/null

echo "source /root/ws_moveit/install/setup.sh" >> /root/.bashrc

if ! command -v pio &> /dev/null; then
  sudo apt update
  sudo apt install -y python3-venv

  # https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html
  wget -qO /tmp/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
  # It MUST be run on disk for some reason.
  python3 /tmp/get-platformio.py

  # https://docs.platformio.org/en/latest/core/installation/shell-commands.html#method-2
  sudo ln -s ~/.platformio/penv/bin/platformio /usr/local/bin/platformio
  sudo ln -s ~/.platformio/penv/bin/pio /usr/local/bin/pio
  sudo ln -s ~/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb

  # https://docs.platformio.org/en/latest/core/installation/udev-rules.html
  sudo mkdir -p /etc/udev/rules.d
  sudo wget -qO /etc/udev/rules.d/99-platformio-udev.rules https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules
fi

echo "NOTE: If using Devcontainers, follow this guide on the host system too:"
echo "https://docs.platformio.org/en/latest/core/installation/udev-rules.html"
