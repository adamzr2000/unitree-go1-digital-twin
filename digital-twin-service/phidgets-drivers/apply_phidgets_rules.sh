sudo cp rules/* /etc/udev/rules.d/
sudo chmod +755 /etc/udev/rules.d/99-phidgets.rules
sudo udevadm control --reload-rules && sudo udevadm trigger