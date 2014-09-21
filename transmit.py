import serial,os
import crcmod
import time
import string,sys
import traceback

CALL = "MYCALL" # My callsign
DOLLARS = 5 # Amount of dollar chars at the beginning of a position packet
POSEVERY = 5 # Amount of images packets transmitted between position packet
imageWidth = 480 # Image width
imageHeight = 256 # Image height
jpegQuality = 50 # JPEG quality
SSDVBAUD = 600 # Baudrate of packet transmission

id = 1 # Counter of image
imagepackets = 0 # Counter of image packets
countPosition = 1 # Counter of position packets

data = {}


#---------------------------------------------------------------------------------

def toDecimalDegrees(ddmm):
	deglen = len(ddmm.split(b'.')[0])-2
	d = int(ddmm.split(b'.')[0][0 : deglen])
	m = float(ddmm[deglen:]) / 60
	return d+m

def _float(s):
    """
    Returns the float value of string s if it exists,
    or None if s is an empty string.
    """
    if s:
        return float(s)
    else:
        return None


def _int(s):
    """
    Returns the int value of string s if it exists,
    or None if s is an empty string.
    """
    if s:
        return int(s)
    else:
        return None


def calcCheckSum(line):
    """
    Returns the checksum as a one byte integer value.
    In this case the checksum is the XOR of everything after '$' and before '*'.
    """
    s = 0
    for c in line[1:-3]:
        s = s ^ c
    return s


def parseGGA(fields):
    """
    Parses the Global Positioning System Fix Data sentence fields.
    Stores the results in the global data dict.
    """
    # GGA has 15 fields
    assert len(fields) == 15
    data = {}
    # MsgId = fields[0]
    data['UtcTime'] = fields[1]
    data['Latitude'] = toDecimalDegrees(fields[2])
    data['NsIndicator'] = fields[3]
    data['Longitude'] = toDecimalDegrees(fields[4])
    data['EwIndicator'] = fields[5]
    data['PositionFix'] = fields[6]
    data['SatellitesUsed'] = _int(fields[7])
    data['HorizontalDilutionOfPrecision'] = _float(fields[8])
    data['MslAltitude'] = _float(fields[9])
    data['MslAltitudeUnits'] = fields[10]
    data['GeoidSeparation'] = _float(fields[11])
    data['GeoidSeparationUnits'] = fields[12]
    data['AgeOfDiffCorr'] = _float(fields[13])
    data['DiffRefStationId'] = fields[14]
	
    # Attend to lat/lon plus/minus signs
    if data['NsIndicator'] == b'S':
        data['Latitude'] *= -1.0
    if data['EwIndicator'] == b'W':
        data['Longitude'] *= -1.0
    return data

def parseLine(line):
	"""
	Parses an NMEA sentence, sets fields in the global structure.
	Raises an AssertionError if the checksum does not validate.
	Returns the type of sentence that was parsed.
	"""
	# Get rid of the \r\n if it exists
	line = line.rstrip()

	# Validate the sentence using the checksum
	if calcCheckSum(line) != int(line[-2:], 16):
		print("[checksum wrong]")

	# Pick the proper parsing function
	try:
		data = parseGGA(line[:-3].split(b','))
		return line[:-3].split(b','), data
		
	except Exception as e:
		#traceback.print_exc()
		return 0, None
    


def getField(fieldname):
    """
    Returns the value of the named field.
    """
    return data[fieldname]

#---------------------------------------------------------------------------------

def crc16_ccitt(data):
    crc16 = crcmod.predefined.mkCrcFun('crc-ccitt-false')
    return hex(crc16(data))[2:].upper().zfill(4)

def prepare_packet(count, timestr, lat, lon, alt, satellites):
	timestr = time.strftime("%H:%M:%S")
	data = "{},{},{},{},{},{},{}".format(CALL, count, timestr, round(lat,5), round(lon,5), round(alt), satellites)
	packet = "{}{}*{}\n".format("$"*DOLLARS, data, crc16_ccitt(data.encode("ascii")))
	return packet.encode("ascii")

class SS(object):
        def __exit__(self, type, value, traceback):
                self.ser.flush()
                self.ser.close()

        def write(self, s):
                if not self.ser.isOpen():
                        print("serial port not open")
                self.ser.write(s)

        def readline(self):
                return self.ser.readline()

class GPS(SS):
	def __enter__(self):
                self.ser = serial.Serial(port = "/dev/ttyAMA0", baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=10)
                return self.ser

class Transmitter(SS):
        def __enter__(self):
                self.ser = serial.Serial(port = "/dev/ttyAMA0", baudrate=SSDVBAUD, bytesize=8, parity='N', stopbits=2)
                return self.ser



def transmitPosition():
	global countPosition
	#Close transmission serial & open GPS serial

	#Catch GPS GGA string
	gga = 0
	
	waitforlock = 60
	with GPS() as ser:
		print("Waiting for lock", end="")
		sys.stdout.flush()
		while gga == 0:
			if waitforlock <= 0:
				print("...no lock")
				return # lock timeout
			try:
				line = ser.readline()
			except:
				return # timeout
			gga, data = parseLine(line)
			print("...{}".format(waitforlock), end="")
			sys.stdout.flush()
			waitforlock -= 1
	print("...lock!")	
	#gga, data = parseLine(b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47")
	print(gga)

	#Close GPS serial & open transmission serial
	with Transmitter() as ser:
		#Write Position
		ser.write(prepare_packet(countPosition, data['UtcTime'], data['Latitude'], data['Longitude'], data['MslAltitude'], data['SatellitesUsed']))
		print("Transmitted Position")

	countPosition += 1

def high_alt_settings():
	"""
	Transmits information to ublox MAX/ or MAX8 to use high altitude settings.
	This is necessary to make it work up to 50km altitude.
	"""
	p = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 
	0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 
	0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x16, 0xDC]
	with GPS() as ser:
		ser.write(bytes(p))


while True:
	print("Take picture")
	os.system('raspistill -w ' + imageWidth + ' -h ' + imageHeight + ' -t 100 -q ' + jpegQuality + ' -vs -o /home/pi/ssdv/bin/image.jpg')

	print("Convert picture into SSDV [ID=" + str(id) + "]")
	os.system('/home/pi/ssdv/bin/ssdv -e -t ' + jpegQuality + ' -c ' + CALL + ' -i ' + str(id) + ' /home/pi/ssdv/bin/image.jpg /home/pi/ssdv/bin/image.ssdv')

	high_alt_settings() # Transmit GPS settings to GNSS module

	print("Open SSDV file")
	f = open('/home/pi/ssdv/bin/image.ssdv', 'rb')
	packets = os.path.getsize('/home/pi/ssdv/bin/image.ssdv') >> 8
	
	print("Transmit " + str(packets) + " packets")

	try:
		for i in range(packets):
			print("Transmit packet #" + str(i+1))
			with Transmitter() as ser:
				ser.write(f.read(256))
			imagepackets += 1
			
			if imagepackets % POSEVERY == 0: #Transmit position
				try:
					transmitPosition()
				except Exception as e:
					print(e)
	except Exception as e:
		traceback.print_exc()
	finally:
		f.close()
		  
	
	id += 1
