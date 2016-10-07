homegear-enocean-tests
======================

This program automatically tests the correct reception and conversion of all possible values of all EEP sensors in Homegear.

Requirements:

- EnOcean USB 300 to send packets.
- Homegear with EnOcean enabled and working.

Usage:

- Compile by executing "make.sh".
- Execute "homegear-enocean-tests SERIALDEVICE ENOCEAN_INTERFACE_NAME" where SERIALDEVICE is the path to your USB 300 and ENOCEAN_INTERFACE_NAME is the name of the USB 300 as defined in "/etc/homegear/families/enocean.conf". On test errors the program exits with non zero exit code.