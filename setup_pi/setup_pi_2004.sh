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

sudo rm /etc/apt/sources.list/d/ros-latest.list
sudo rm /etc/apt/sources.list/d/ros2.list

sudo apt update

curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh ./get-docker.sh
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
