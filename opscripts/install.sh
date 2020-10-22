#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
sudo apt remove modemmanager -y
sudo mkdir -p /etc/roverrobotics
sudo cp $DIR/env.sh /etc/roverrobotics/env.sh
sudo cp $DIR/roscore.service /etc/systemd/system/roscore.service
sudo cp $DIR/roverrobotics.service /etc/systemd/system/roverrobotics.service
sudo cp $DIR/roverrobotics /usr/sbin/roverrobotics
sudo systemctl enable roverrobotics.service
sudo systemctl enable roscore.service
sudo chmod +x /usr/sbin/roverrobotics
