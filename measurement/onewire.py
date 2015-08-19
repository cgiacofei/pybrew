"""
Download the device tree overlay package use Robert C Nelson's source install:
https://raw.github.com/RobertCNelson/tools/master/pkgs/dtc.sh

After installing the software, make the file executable, and then run the bash
file to install device-tree-overlay (dtc) from the source:
> sudo chmod +x dtc.sh
> sudo bash dtc.sh

Compile the .dtbo file:
> dtc -O dtb -o BB-W1-00A0.dtbc -b o -@ BB-W1-00A0.dts

Copy the .dtbo file to /lib/firmware:
> cp BB-W1-00A0.dtbc /lib/firmware

Mux the pins (this must be done on every reboot):
> sudo echo BB-W1:00A0 > /sys/devices/bone_capemgr.9/slots

"""

import os
os.system("echo BB-W1:00A0 > /sys/devices/bone_capemgr.9/slots")

class OneWire(object):
    """
        address: device address (ex. 28-00000494acf0/w1_slave)
    """

    def init(self, address, path='/sys/bus/w1/devices/'):
        self.device_path = path
        self.device_file = ''.join([self.device_path, address])


    def list_devices(self):
        contents = os.listdir(self.device_path)

        dev = [f for f in contents if os.path.isdir(os.path.join(self.device_path,f))]

        return dev
