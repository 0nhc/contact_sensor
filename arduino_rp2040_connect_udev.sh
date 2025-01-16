echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="005e", MODE:="0777", SYMLINK+="arduino_rp2040_connect"' >/etc/udev/rules.d/arduino_rp2040_connect.rules
sudo chmod 644 /etc/udev/rules.d/arduino_rp2040_connect.rules
sudo udevadm control --reload
sleep 2
sudo udevadm trigger
