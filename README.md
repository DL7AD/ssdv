SSDV usage script
====

This is an usage script for fsphil's SSDV encoder which can be runned on a RaspberryPI.

Supported
- SSDV (Digital image transmission)
- Position transmission (ublox MAX7 or MAX8 GNSS modules)
- RaspberryPI Camera
- Radiometrix NTX2b

Informations
- The software uses the binaries of fsphil's SSDV encoding software which can be found here: https://github.com/fsphil/ssdv
  The file 'ssdv' which can be found in this repository is an build for the RaspberryPI. If you want to run it on another
  system, please go to fsphil's repository and get the source files.
- Python3 necessary

Recommondations
- Use 300 baud in a not well covered region. That will make the pictures more reliable.

What this software does
====
The software takes a picture by the RaspberryPI camera and converts it into the SSDV format.
It will then transmit the data by the serial port (/dev/ttyAMA0) which has to be connected with an Radiometrix NTX2b.
The GNSS is also connected to the same serial port. When the GNSS position is captured by the serial port, there can't
take place a transmission. There must be the RXD and TXD serial ports connected to the GNSS module to make it possible
to set it into High Altitude mode. Otherwise the GNSS module will stop working at 12km altitude!
Every 5 image packets there will be a position packet transmitted. This value can be changed. If the GNSS has no lock
it will continue on transmitting image packets.
The image packets can be received by an RTL dongle and decoded by DL-FLDigi.

Links
- http://ukhas.org.uk/guides:tracking_guide
- http://ukhas.org.uk/guides:ssdv
- https://github.com/fsphil/ssdv
- www.spacenear.us/tracker
- http://ssdv.habhub.org
