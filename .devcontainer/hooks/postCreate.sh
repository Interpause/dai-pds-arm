#!/usr/bin/env bash
# Insert commands to run when container is started for the first time.

# Allow apt index & package cache to persist.
sudo rm -f /etc/apt/apt.conf.d/docker-clean
echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' | sudo tee /etc/apt/apt.conf.d/keep-cache > /dev/null

echo "source /root/ws_moveit/install/setup.sh" >> /root/.bashrc
sudo apt-get install -y python3-venv
