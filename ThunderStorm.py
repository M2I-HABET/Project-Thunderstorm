# Write your code here :-)
# Simple GPS module demonstration.
# Will wait for a fix and print a message every second with the current location
# and other details.
import time
import board
import busio
import digitalio

import adafruit_lsm9ds1
from busio import I2C
import adafruit_bme680
import adafruit_gps
import adafruit_rfm9x
import adafruit_max31865
 # Device ID
FEATHER_ID = b'4'

print("start up")
# Define pins connected to the chip, use these if wiring up the breakout according to the guide:
# pylint: disable=c-extension-no-member
CS = digitalio.DigitalInOut(board.D10)
HR = digitalio.DigitalInOut(board.A0)

# pylint: disable=c-extension-no-member
RESET = digitalio.DigitalInOut(board.D11)

# Define the onboard LED
LED = digitalio.DigitalInOut(board.D13)
LED.direction = digitalio.Direction.OUTPUT

# Initialize SPI bus.
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)


# Define radio frequency, MUST match gateway frequency.
RADIO_FREQ_MHZ = 433.0

# Initialze RFM radio
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# Set transmit power to max
rfm9x.tx_power = 23

# Define RX and TX pins for the board's serial port connected to the GPS.
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
RX = board.RX
TX = board.TX

# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
uart = busio.UART(TX, RX, baudrate=9600, timeout=1)
#uartPayload = busio.UART(board.A1, RX, baudrate=9600, timeout=30)
# for a computer, use the pyserial library for uart access
#import serial
#uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=3000)

# Create a GPS module instance.
#gps = adafruit_gps.GPS(uart, debug=False)

# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
#   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

# Turn on the basic GGA and RMC info (what you typically want)
#gps.send_command(b'PMTK314,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn on just minimum info (RMC only, location):
#gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Turn off everything:
#gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Tuen on everything (not all of it is parsed!)
#gps.send_command(b'PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0')

# Set update rate to once a second (1hz) which is what you typically want.
#gps.send_command(b'PMTK220,1000')
# Or decrease to once every two seconds by doubling the millisecond value.
# Be sure to also increase your UART timeout above!
#gps.send_command(b'PMTK220,2000')
# You can also speed up the rate, but don't go too fast or else you can lose
# data during parsing.  This would be twice a second (2hz, 500ms delay):
#gps.send_command(b'PMTK220,500')

# Main loop runs forever printing the location, etc. every second.
last_print = time.monotonic()


# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
#sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Create library object using our Bus I2C port
#i2c = I2C(board.SCL, board.SDA)
bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, debug=False)

# change this to match the location's pressure (hPa) at sea level
bme680.sea_level_pressure = 980

#SPI connection:
# from digitalio import DigitalInOut, Direction
# spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
# csag = DigitalInOut(board.D5)
# csag.direction = Direction.OUTPUT
# csag.value = True
# csm = DigitalInOut(board.D6)
# csm.direction = Direction.OUTPUT
# csm.value = True
# sensor = adafruit_lsm9ds1.LSM9DS1_SPI(spi, csag, csm)

# Main loop will read the acceleration, magnetometer, gyroscope, Temperature
# values every second and print them out.


pt100 = adafruit_max31865.MAX31865(spi, HR, rtd_nominal=100, ref_resistor=430.0, wires=3)


def sendMessage(message):
    try:
        LED.value = True
        rfm9x.send(FEATHER_ID+b','+message)
        #print(FEATHER_ID+b','+message)
        time.sleep(.5)
        LED.value = False
    except:
        print("Message failed to send")

def getDat():
    temp = pt100.temperature
    RTemp = str('{0:0.3f}'.format(temp))
    B = 'ResistTemp,'+ RTemp
    t = str(bme680.temperature)
    g = str(bme680.gas)
    h = str(bme680.humidity)
    p = str(bme680.pressure)
    a = str(bme680.altitude)

    A = t + ',' + g + ',' + h + ',' + p + ',' + a

    datstr = "DATA,BME," + A + "," + B + ",HABET" + ",eol"
    return(datstr)

current = time.monotonic()
old = current
newGPS = False
while True:
    # Make sure to call gps.update() every loop iteration and at least twice
    # as fast as data comes from the GPS unit (usually every second).
    # This returns a bool that's true if it parsed new data (you can ignore it
    # though if you don't care and instead look at the has_fix property).
    #gps.update()
    # Every second print out current location details if there's a fix.



    # Read acceleration, magnetometer, gyroscope, temperature.
    #accel_x, accel_y, accel_z = sensor.acceleration
    #mag_x, mag_y, mag_z = sensor.magnetic
    #gyro_x, gyro_y, gyro_z = sensor.gyro
    # Print values.
    #print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    #print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
    #print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
    #print('Temperature: {0:0.3f}C'.format(temp))
    #time.sleep(1.0)
    #aX = str('{0:0.3f}'.format(accel_x))
    #aY = str('{0:0.3f}'.format(accel_y))
    #aZ = str('{0:0.3f}'.format(accel_z))
    #mX = str('{0:0.3f}'.format(mag_x))
    #mY = str('{0:0.3f}'.format(mag_y))
    #mZ = str('{0:0.3f}'.format(mag_z))
    #gX = str('{0:0.3f}'.format(gyro_x))
    #gY = str('{0:0.3f}'.format(gyro_y))
    #gZ = str('{0:0.3f}'.format(gyro_z))
    #gZ = str('{0:0.3f}'.format(gyro_z))
    #B = 'acc,' + aX + ',' + aY + ',' + aZ + ',mag,' + mX + ',' + mY + ',' + mZ + ',gyro,' + gX + ',' + gY + ',' + gZ
    #print(B)
    # Delay for a second.

    #print("\nTemperature: %0.1f C" % bme680.temperature)
    #print("Gas: %d ohm" % bme680.gas)
    #print("Humidity: %0.1f %%" % bme680.humidity)
    #print("Pressure: %0.3f hPa" % bme680.pressure)
    #print("Altitude = %0.2f meters" % bme680.altitude)

    #t = bme680.temperature
    #g = bme680.gas

    #sensorString = "str(t),str(g)"
    #print(sensorString)

    #time.sleep(1)
    #time.sleep(1)

    #A = [t, g, h, p, a]
    #print(A)




    current = time.monotonic()
    #if current - last_print >= 1.0:
    if uart.in_waiting > 0:
        gps_string = uart.readline()
        print(gps_string)
        if "GPGGA" in gps_string:
            #old = current
            gps_str = str(gps_string)
            #print(gps_str)
            #sendMessage(gps_str + ",HABET")
            #datastr = getDat()
            #sendMessage(datastr)
            #print(datastr)
            newGPS = True
    if current-old>5:
        old = current
        if newGPS:
            print(gps_str)
            sendMessage(gps_str + ",HABET")
            datastr = getDat()
            sendMessage(datastr)
            print(datastr)
            newGPS = False
        else:
            sendMessage("No GPS,HABET")
            print("No GPS")
            datastr = getDat()
            sendMessage(datastr)
            print(datastr)
    packet = rfm9x.receive(timeout=.1)
    if packet is None:
        # Packet has not been received
        LED.value = False
        #print('Received nothing! Listening again...')
    else:
        # Received a packet!
        LED.value = True
        # Print out the raw bytes of the packet:
        #print('Received (raw bytes): {0}'.format(packet))
        # And decode to ASCII text and print it too.  Note that you always
        # receive raw bytes and need to convert to a text format like ASCII
        # if you intend to do string processing on your data.  Make sure the
        # sending side is sending ASCII data before you try to decode!
        try:
            rssi = rfm9x.rssi
            #print(packet)
            if(b'command' in packet):
                sendMessage(str(packet))
                print(str(packet))
        except:
            print("invalid ascii")
        # Also read the RSSI (signal strength) of the last received message and
        # print it.

    time.sleep(.05)
