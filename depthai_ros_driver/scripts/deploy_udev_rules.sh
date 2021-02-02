#!/bin/bash

[ "$UID" -eq 0 ] || exec sudo "$0" "$@"

cat > /etc/udev/rules.d/80-movidius.rules  <<EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
EOF

udevadm control --reload-rules
udevadm trigger

echo "Udev rules deployed successfully, if depthai is connected, please remove and reconnect."
