#!/bin/bash

# 定义规则文件路径
RULES_FILE="/etc/udev/rules.d/robot.rules"

# 写入规则
echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="0000", SYMLINK+="robot_can", MODE:="0666"' | sudo tee $RULES_FILE

# 重新加载udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger
