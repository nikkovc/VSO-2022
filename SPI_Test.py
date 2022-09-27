import spidev
from time import sleep

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.xfer2([0x88, 0x03])

sleep(.1)

spi.xfer2([0x90, 0x00])