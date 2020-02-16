# Circuitpyton driver for MS5637 digital barometric temperature and pressure sensor over I2C
"""
`ms5637`

================================================================================

CircuitPython helper library for the MS5637 temperature and pressure sensor

* Author(s): Brian Dahlem

Implementation Notes
--------------------

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases


* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

from micropython import const
import time
import adafruit_bus_device.i2c_device as i2c_device

# list of commands in hex for MS5637
_MS5637_reset = const(0x1E) # reset command
_MS5637_C1 = const(0xA2) # read PROM C1 command
_MS5637_C2 = const(0xA4) # read PROM C2 command
_MS5637_C3 = const(0xA6) # read PROM C3 command
_MS5637_C4 = const(0xA8) # read PROM C4 command
_MS5637_C5 = const(0xAA) # read PROM C5 command
_MS5637_C6 = const(0xAC) # read PROM C6 command
_MS5637_adc = const(0x00) # read ADC command
_MS5637_start_d1 = const(0x4A) # convert D1 (OSR=8192)
_MS5637_start_d2 = const(0x5A) # convert D2 (OSR=8192)

_MS5637_address = 0x76

class MS5637:
	"""Driver for the MS5637 Pressure Sensor
		:param ~busio.I2C i2c_bus: the I2C buss the MS5637 is connected to.
	"""
	
	def __init__(self, i2c_bus):
		self.i2c_device = i2c_device.I2CDevice(i2c_bus, _MS5637_address)
		self.reset()
		self._c1 = self.readProm(_MS5637_C1)*65536
		self._c2 = self.readProm(_MS5637_C2)*131072
		self._c3 = self.readProm(_MS5637_C3)/128
		self._c4 = self.readProm(_MS5637_C4)/64
		self._c5 = self.readProm(_MS5637_C5)*256
		self._c6 = self.readProm(_MS5637_C6)/8388608
		
	def reset(self):
		""" reset the device to ensure PROM is loaded """
		with self.i2c_device:
			self.i2c_device.write(bytearray([_MS5637_reset]))
			
	def readProm(self, address):
		data = bytearray(2)
		with self.i2c_device:
			self.i2c_device.write_then_readinto(bytearray([address]), data)
			return int.from_bytes(data, "big")
			
	@property
	def temperature(self):
		temp, dT = self._temp()
		return temp
	
	def _temp(self):
		# start adc
		with self.i2c_device:
			self.i2c_device.write(bytearray([_MS5637_start_d2]))
			
		time.sleep(0.02)
		
		data = bytearray(3)
		with self.i2c_device:
			self.i2c_device.write_then_readinto(bytearray([_MS5637_adc]), data)
		
		raw_temp = int.from_bytes(data, "big")
		dT = raw_temp - (self._c5) # difference between actual and ref temp
		temp = (2000 + (dT * self._c6))/100  # actual temperature
		return (temp, dT)		
	
	@property
	def pressure(self):
		temp, dT = self._temp()
		
		# start adc
		with self.i2c_device:
			self.i2c_device.write(bytearray([_MS5637_start_d1]))
			
		time.sleep(0.02)
		
		data = bytearray(3)
		with self.i2c_device:
			self.i2c_device.write_then_readinto(bytearray([_MS5637_adc]), data)
		
		raw_adc = int.from_bytes(data, "big")
		off = self._c2 + (self._c4 * dT) # offset at actual temperature
		sense = self._c1 + (self._c3 * dT) # pressure offset at actual temperature
		return (raw_adc * sense / 2097152 - off) / 3276800 # barometric pressure
