#HABET Thunderstorm RF300 script
#This script reads data from the on board sensors and 
#sends the data out to the network.
#Matthew E. Nelson

#Revision - 0.1

#Revision History
# 0.1 - Initial commit

#includes the correct GPIO pin map
from synapse.platforms import *

#Defines
HIH_5030 = GPIO_09 # Analog out from HIH-5030 sensor
Data_Ready = GPIO_10 # Data Ready pin from pressure sensor
ADC_CHAN = 3

@setHook(HOOK_STARTUP)
def startup():
    initProtoHw()
    monitorPin(BUTTON_PIN, True)
    setPinDir(GPS_ENABLE_PIN, True)
    writePin(GPS_ENABLE_PIN, True)
    initUart(0, 4800)
    stdinMode(1, False)   # Char mode, no echo
    
    # Connect GPS serial output to STDIN, where our event handler will parse the messages
    crossConnect(DS_UART0, DS_STDIO)