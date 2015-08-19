
from onewire import OneWire

class DS18B20(OneWire):
    """1 Wire class for DS18B20 Temperature sensor"""

    def read_temp(self):
        """Return temperature in deg C"""

        temp = ""
        raw = open(device_file, "r").read()
        temp = float(raw.split("t=")[-1])/1000

        return temp
