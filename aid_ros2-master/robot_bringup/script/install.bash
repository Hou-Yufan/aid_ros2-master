#!/bin/bash
sudo cp 01-netcfg.yaml /etc/netplan
sudo netplan apply
sudo cp ros_launch.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable ros_launch.service
sudo systemctl start ros_launch.service

