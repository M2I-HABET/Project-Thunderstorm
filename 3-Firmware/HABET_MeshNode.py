##############################################################################
#
# HABET_MeshNode.py
#
# This script is designed to be run on a RF300 module to provide periodic
# link quality information with the node's current position.
#
# Author: Ian McInerney
##############################################################################


##############################################################################
# Various global variable initialization stuff
##############################################################################

# Define the multicast group that the node belongs to
MULTICAST_GROUP = 0x0002

# Store if there is currently a GPS string being buffered
# 0 = No string
# 1 = Incomplete string
# 2 = Complete string
gpsStringBuffered = 0

# The buffer for the GPS
gpsStringBuffered = {}

# The GPS decoded variables
gps_Latitude = -1
gps_Longitude = -1
gps_Altitude = -1
gps_Time = -1
gps_Satellites = -1


##########################################################################
# Run this code at startup
##########################################################################

@setHook(HOOK_STARTUP)
def _startupEvent():
    # Configure the multicast groups
    saveNvParam(5, MULTICAST_GROUP)
    saveNvParam(6, MULTICAST_GROUP)

    #configure the serial port for the GPS to go to the script
    initUart(0, 9600)
    crossConnect(1, 4) # Connect UART0 to STDIO
    

########################################################################
# This function is called whenever new data appears on the serial input
########################################################################

@setHook(HOOK_STDIN)
def _receiveSerialData(data):
    global gpsStringBuffered
    global gpsStringBuffer
    
    # Find the number of characters received
    receivedLength = len(data)
    
    if gpsStringBuffered == 0:
        # No string is buffered, search for the start character
        charLocation = _stringFindNextInstance(data, "$", 0)
        
        if( charLocation == -1):
            # No start character was found
            return
        
        # A start character exists, pull the string out
        gpsStringBuffer = data[charLocation+1:receivedLength]
        
        # We are now buffering a GPS string
        gpsStringBuffered = 1;
        
    elif gpsStringBuffered == 1:
        # A string is being bufferef, search for the stop character
        charLocation = _stringFindNextInstance(data, "*", 0)
        
        if( charLocation == -1):
            # No stop character was found, just add the entire string
            gpsStringBuffer = gpsStringBuffer + data
            
        else:
            # A stop character was found, only add the part before it
            gpsStringBuffer = gpsStringBuffer + data[:charLocation-1]
            gpsStringBuffered = 2
            
            _decodeGPS()            # Decode the GPS string
            gpsStringBuffered = 0   # There is no more string in the buffer
        



#################################################################
# These functions serve as the periodic data generation functions
#################################################################

# Call this function every second
@setHook(HOOK_1S)
def _secondTick():
    global tickCount
    tickCount = tickCount+1
    if tickCount == 10:
        # Every 10 seconds, send the link status
        _sendLinkStatus()
        tickCount = 0

# Send a link status update packet to each node in listening range (no hopping)
def _sendLinkStatus():
    global gps_Latitude
    global gps_Longitude
    global gps_Altitude
    global gps_Time
    global gps_Satellites

    #Send the most recent GPS location to each node in listening range
    mcastRpc(MULTICAST_GROUP, 1, 'receiveLinkStatus', gps_Time, gps_Latitude, gps_Longitude, gps_Altitude, gps_Satellites)
    

##########################################################################
# This function is called by remote nodes to do link information gathering
##########################################################################

# This function is called by remote nodes to update link status
def receiveLinkStatus(rec_gps_Time, rec_gps_Latitude, rec_gps_Longitude, rec_gps_Altitude, rec_gps_Satellites):
    receivedQuality = getLq()   # Read in the link quality right away
    sendingNode = rpcSourceAddr() # Get which node sent the RPC
    



#############################################################################
# The following functions are used to parse the GPS string from the Antennova
#############################################################################

# Break up a GPS string into its parts
def _decodeGPS(inputString):
    global gps_Latitude
    global gps_Longitude
    global gps_Altitude
    global gps_Time
    global gps_Satellites
    
    stringLength = len(inputString)
    
    # Return if it is not a GGA string
    if (inputString[:4] != "GPGGA"):
        return
    
    # Start after the "GPGGA," part
    i = 6;
    currentItem = 0;
    
    while( i < stringLength ):
        # Find the next comma after the current location
        nextComma = _stringFindNextInstance(inputString, ",", i)
        
        # Extract the substring from the string and store it appropriately
        if (currentItem == 0):
            # The GPS time string
            end = nextComma-1
            gps_Time = inputString[i+1:end]
            
        elif (currentItem == 1):
            # The GPS Latitude String
            # Note: This has two fields, the second is a single letter
            end = nextComma+1
            gps_Latitude = inputString[i+1:end]
            
        elif (currentItem == 2):
            # The GPS Longitude String
            # Note: This has two fields, the second is a single letter
            end = nextComma+1
            gps_Longitude = inputString[i+1:end]
            
        elif (currentItem == 4):
            # The GPS satellite count
            end = nextComma-1
            gps_Satellites = inputString[i+1:end]
            
        elif (currentItem == 6):
            # The GPS reported altitude
            # Note: This has two fields, the second is a single letter
            end = nextComma+1
            gps_Altitude = inputString[i+1:end]
        else:
            end = nextComma
            
        # Move onto the next field
        currentItem += 1
        
        # Set the index to be the start of the next field
        i = end+2
    


# Find the next instance of a character after a certain point
def _stringFindNextInstance(inputString, desiredChar, startIndex):
    inputLen = len(inputString)
    i = startIndex;
    
    # Iterate over the string's length
    while (i < inputLen):
        if (inputString[i] == desiredChar):
            # Record the location of the hit
            return(i)
        i += 1
    
    # If nothing is found, return -1
    return(-1)