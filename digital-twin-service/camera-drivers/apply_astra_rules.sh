sudo cp rules/* /etc/udev/rules.d/
sudo chmod +755 /etc/udev/rules.d/56-orbbec-usb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
