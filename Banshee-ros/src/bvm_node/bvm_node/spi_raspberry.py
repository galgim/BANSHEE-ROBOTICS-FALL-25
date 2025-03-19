import spidev
import time

spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 100000
spi.mode = 0


try:
    while True:
        send_byte = 0x26
        received_byte = spi.xfer2([send_byte])[0]
        print(f"Sent: 0x{send_byte:X}, Received: 0x{received_byte:X}")
        time.sleep(1)
except KeyboardInterrupt:
    spi.close()