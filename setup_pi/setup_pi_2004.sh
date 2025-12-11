#!/usr/bin/env bash

set -eux -o pipefail

if [[ $# -le 1 ]]; then
    echo "Incorrect number of arguments. "
    echo $#
    echo " Usage: 'setup_pi.sh mycobot_23 secret_password'"
    exit 1
fi

WLAN_SSID=$1
WLAN_PASS=$2

sudo rm /etc/apt/sources.list.d/ros-latest.list
sudo rm /etc/apt/sources.list.d/ros2.list

sudo apt update -y

# Add Docker's official GPG key:
sudo apt install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo usermod -aG docker $USER
sudo docker pull mzandtheraspberrypi/ros_sim:2024-10-24

mkdir -p /home/er/ros_ws/src

# https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nm-settings-nmcli.html
# useful options. We want to setup a wifi network and let folks connect to it and get an ip address
nmcli d wifi hotspot ifname wlan0 ssid ${WLAN_SSID} password ${WLAN_PASS} con-name my-hotspot
nmcli c down my-hotspot
nmcli c modify my-hotspot connection.autoconnect yes
nmcli c modify my-hotspot connection.autoconnect-priority 999

nmcli c up my-hotspot

sudo reboot
