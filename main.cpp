#include <homegear-base/BaseLib.h>
#include <string>
#include <iostream>
#include <vector>
#include <memory>

std::shared_ptr<BaseLib::SerialReaderWriter> _serial;
uint32_t _intAddress = 0;
std::vector<char> _byteAddress;
std::string _enoceanInterface;

uint8_t crc8Table[256] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
	0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
	0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
	0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
	0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
	0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
	0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
	0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
	0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
	0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
	0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
	0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
	0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
	0x76, 0x71, 0x78, 0x7f, 0x6A, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
	0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
	0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8D, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
	0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

void sendPacket(std::vector<char> data);
std::vector<char> readPacket();
void getAddress();
uint64_t createDevice(std::string eep);
void deleteDevice(uint64_t peerId);
int64_t getIntValue(uint64_t peerId, int32_t channel, std::string variable);
bool getBooleanValue(uint64_t peerId, int32_t channel, std::string variable);
double getDoubleValue(uint64_t peerId, int32_t channel, std::string variable);
void setValue(uint64_t peerId, int32_t channel, std::string variable, bool value);
void setValue(uint64_t peerId, int32_t channel, std::string variable, int32_t value);
void runTests();
void testF6();
void testD5();
void testA5();
void testD2();

void printHelp()
{
	std::cout << "Usage: homegear-enocean-tests SERIALDEVICE INTERFACENAME" << std::endl;
	std::cout << "  SERIALDEVICE:   The device name of the USB 300 used for sending test packets (Example: \"/dev/ttyUSB0\")" << std::endl;
	std::cout << "  INTERFACENAME:  The name of the USB 300 used by Homegear as defined in \"/etc/homegear/families/enocean.conf\" (Example: \"My-EnOcean-Interface\")" << std::endl;
}

void getAddress()
{
	std::vector<char> packet{ 0x55, 0x00, 0x01, 0x00, 0x05, 0x00, 0x08, 0x00 };
	sendPacket(packet);

	int64_t startTime = BaseLib::HelperFunctions::getTime();
	while(BaseLib::HelperFunctions::getTime() < startTime + 2000)
	{
		packet = readPacket();
		if(packet.size() != 13 || packet[4] != 2 || packet[1] != 0 || packet[2] != 5 || packet[3] != 1 || packet[6] != 0) continue;
		_intAddress = ((uint32_t)(uint8_t)packet[7] << 24) | ((uint32_t)(uint8_t)packet[8] << 16) | ((uint32_t)(uint8_t)packet[9] << 8) | (uint8_t)packet[10];
		_byteAddress.resize(4);
		_byteAddress[0] = packet[7];
		_byteAddress[1] = packet[8];
		_byteAddress[2] = packet[9];
		_byteAddress[3] = packet[10];
		std::cout << "EnOcean address is: " << BaseLib::HelperFunctions::getHexString(_byteAddress) << std::endl;
		break;
	}
	if(_intAddress == 0)
	{
		std::cerr << "Could not get EnOcean address from USB 300." << std::endl;
		exit(1);
	}
}

uint64_t createDevice(std::string eep)
{
	std::cout << "Creating device with EEP \"" + eep + "\"... ";
	std::string output;
	BaseLib::HelperFunctions::exec("homegear -e rc 'print($hg->createDevice(15, (int)hexdec(\"" + eep + "\"), \"\", (int)" + std::to_string(_intAddress) + ", 0, \"" + _enoceanInterface + "\"));'", output);
	if(output.find("HomegearException") != std::string::npos)
	{
		std::cerr << "Could not create device. HomegearException thrown: " << output << std::endl;
		exit(1);
	}
	uint64_t peerId = BaseLib::Math::getNumber(output);
	if(peerId == 0)
	{
		std::cerr << "Could not create device. Returned peer ID is invalid." << std::endl;
		exit(1);
	}
	std::cout << "ID: " << peerId << std::endl;
	return peerId;
}

void deleteDevice(uint64_t peerId)
{
	std::cout << "Removing device ... ";
	std::string output;
	BaseLib::HelperFunctions::exec("homegear -e rc '$hg->deleteDevice((int)" + std::to_string(peerId) + ", 0);'", output);
	if(output.find("HomegearException") != std::string::npos)
	{
		std::cerr << "Could not delete device. HomegearException thrown: " << output << std::endl;
		exit(1);
	}
	std::cout << "ok" << std::endl;
}

int64_t getIntValue(uint64_t peerId, int32_t channel, std::string variable)
{
	std::string output;
	BaseLib::HelperFunctions::exec("homegear -e rc 'print($hg->getValue((int)" + std::to_string(peerId) + ", (int)" + std::to_string(channel) + ", \"" + variable + "\"));'", output);
	if(output.find("HomegearException") != std::string::npos)
	{
		std::cerr << "Could not get value of variable \"" + variable + "\" for peer \"" + std::to_string(peerId) + "\" on channel \"" + std::to_string(channel) + "\". HomegearException thrown: " << output << std::endl;
		exit(1);
	}
	return BaseLib::Math::getNumber(output);
}

bool getBooleanValue(uint64_t peerId, int32_t channel, std::string variable)
{
	std::string output;
	BaseLib::HelperFunctions::exec("homegear -e rc 'print($hg->getValue((int)" + std::to_string(peerId) + ", (int)" + std::to_string(channel) + ", \"" + variable + "\"));'", output);
	if(output.find("HomegearException") != std::string::npos)
	{
		std::cerr << "Could not get value of variable \"" + variable + "\" for peer \"" + std::to_string(peerId) + "\" on channel \"" + std::to_string(channel) + "\". HomegearException thrown: " << output << std::endl;
		exit(1);
	}
	return output == "true";
}

double getDoubleValue(uint64_t peerId, int32_t channel, std::string variable)
{
	std::string output;
	BaseLib::HelperFunctions::exec("homegear -e rc 'print($hg->getValue((int)" + std::to_string(peerId) + ", (int)" + std::to_string(channel) + ", \"" + variable + "\"));'", output);
	if(output.find("HomegearException") != std::string::npos)
	{
		std::cerr << "Could not get value of variable \"" + variable + "\" for peer \"" + std::to_string(peerId) + "\" on channel \"" + std::to_string(channel) + "\". HomegearException thrown: " << output << std::endl;
		exit(1);
	}
	return BaseLib::Math::getDouble(output);
}

void setValue(uint64_t peerId, int32_t channel, std::string variable, bool value)
{
	std::string output;
	BaseLib::HelperFunctions::exec("homegear -e rc '$hg->setValue((int)" + std::to_string(peerId) + ", (int)" + std::to_string(channel) + ", \"" + variable + "\", (bool)" + std::to_string(value) + ");'", output);
	if(output.find("HomegearException") != std::string::npos)
	{
		std::cerr << "Could not set value of variable \"" + variable + "\" for peer \"" + std::to_string(peerId) + "\" on channel \"" + std::to_string(channel) + "\". HomegearException thrown: " << output << std::endl;
		exit(1);
	}
}

void setValue(uint64_t peerId, int32_t channel, std::string variable, int32_t value)
{
	std::string output;
	BaseLib::HelperFunctions::exec("homegear -e rc '$hg->setValue((int)" + std::to_string(peerId) + ", (int)" + std::to_string(channel) + ", \"" + variable + "\", (int)" + std::to_string(value) + ");'", output);
	if(output.find("HomegearException") != std::string::npos)
	{
		std::cerr << "Could not set value of variable \"" + variable + "\" for peer \"" + std::to_string(peerId) + "\" on channel \"" + std::to_string(channel) + "\". HomegearException thrown: " << output << std::endl;
		exit(1);
	}
}

std::vector<char> readPacket()
{
	int64_t startTime = BaseLib::HelperFunctions::getTime();
	std::vector<char> packet;
	uint32_t size = 0;
	while(BaseLib::HelperFunctions::getTime() < startTime + 2000)
	{
		char data;
		int32_t result = _serial->readChar(data, 5000000);
		if(result == -1)
		{
			std::cerr << "Error" << std::endl;
			exit(1);
		}
		else if(result == 1)
		{
			size = 0;
			packet.clear();
			continue;
		}

		if(packet.empty() && data != 0x55) continue;

		packet.push_back(data);
		if(size == 0 && packet.size() == 6) size = ((packet[1] << 8) | packet[2]) + packet[3] + 7;
		if(size > 0 && packet.size() == size) return packet;
	}
	return packet;
}

void sendPacket(std::vector<char> data)
{
	uint8_t crc8 = 0;
	for(int32_t i = 1; i < 5; i++)
	{
		crc8 = crc8Table[crc8 ^ (uint8_t)data[i]];
	}
	data[5] = crc8;

	crc8 = 0;
	for(int32_t i = 6; i < data.size() - 1; i++)
	{
		crc8 = crc8Table[crc8 ^ (uint8_t)data[i]];
	}
	data.back() = crc8;

	_serial->closeDevice(); // Reconnect to avoid duty cycle limit
	_serial->openDevice(false, false, false);
	_serial->writeData(data);

	usleep(50000);
}

int main(int argc, char* argv[])
{
	if(argc < 3)
	{
		printHelp();
		exit(1);
	}
	
	std::string serialDevice(argv[1]);
	if(serialDevice.find('/') == std::string::npos)
	{
		std::cerr << "Invalid serial device." << std::endl;
		printHelp();
		exit(1);
	}
	std::cout << "Serial device set to " << serialDevice << std::endl;

	_enoceanInterface = std::string(argv[2]);
	std::cout << "EnOcean interface set to " << _enoceanInterface << std::endl;

	std::unique_ptr<BaseLib::Obj> bl(new BaseLib::Obj(std::string(""), nullptr, false));
	try
	{
		_serial.reset(new BaseLib::SerialReaderWriter(bl.get(), serialDevice, 57600, 0, true, -1));
		_serial->openDevice(false, false, false);

		getAddress();

		runTests();
	}
	catch(BaseLib::Exception& ex)
	{
		std::cerr << ex.what() << std::endl;
		exit(1);
	}
	catch(std::exception& ex)
	{
		std::cerr << ex.what() << std::endl;
		exit(1);
	}

	_serial->closeDevice();

	return 0;
}

void runTests()
{
	testF6();
	testA5();
}

void testA502(std::string eep, int32_t maxIndex, double maxTemperature, double factor)
{
	std::cout << std::endl << "Testing EEP " << eep << "... Values should go from " << std::fixed << std::setprecision(1) << (maxTemperature - ((double)maxIndex / factor)) << "°C to " << maxTemperature << "°C... " << std::endl;
	uint64_t peerId = createDevice(eep);
	sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, (char)(uint8_t)0x0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
	sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, (char)(uint8_t)0x0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
	sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, (char)(uint8_t)0x0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
	sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
	sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
	sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
	if(getDoubleValue(peerId, 1, "TEMPERATURE") != maxTemperature)
	{
		deleteDevice(peerId);
		std::cerr << "Wrong value returned" << std::endl;
		exit(1);
	}
	int32_t retries = 5;
	for(int32_t i = maxIndex; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)(i >> 8), (char)(uint8_t)(i & 0xFF), 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value = std::lround((maxTemperature - getDoubleValue(peerId, 1, "TEMPERATURE")) * factor);
		if(value != i)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong value returned for binary value " << i << ": " << value << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}
	deleteDevice(peerId);
}

void testA50401()
{
	std::cout << std::endl << "Testing EEP A50401... Values should go from 0% to 100% and from 0°C to 40°C... " << std::endl;
	uint64_t peerId = createDevice("A50401");

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "TEMPERATURE") != 0 || getDoubleValue(peerId, 1, "HUMIDITY") != 0)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	// {{{ Temperature data available?
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFA, (char)(uint8_t)0xFA, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFA, (char)(uint8_t)0xFA, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFA, (char)(uint8_t)0xFA, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "TEMPERATURE") != 0 || getDoubleValue(peerId, 1, "HUMIDITY") != 100)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (2)" << std::endl;
			exit(1);
		}
	// }}}

	int32_t retries = 5;
	for(int32_t i = 250; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)i, (char)(uint8_t)i, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t temperatureValue = std::lround(getDoubleValue(peerId, 1, "TEMPERATURE") * 6.25);
		int32_t humidityValue = std::lround(getDoubleValue(peerId, 1, "HUMIDITY") * 2.5);
		if(temperatureValue != i || humidityValue != i)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << temperatureValue << ", " << humidityValue << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}
	deleteDevice(peerId);
}

void testA50402()
{
	std::cout << std::endl << "Testing EEP A50402... Values should go from 0% to 100% and from -20°C to 60°C... " << std::endl;
	uint64_t peerId = createDevice("A50402");

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFA, (char)(uint8_t)0xFA, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFA, (char)(uint8_t)0xFA, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFA, (char)(uint8_t)0xFA, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "TEMPERATURE") != -20 || getDoubleValue(peerId, 1, "HUMIDITY") != 0)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	// {{{ Temperature data available?
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFA, (char)(uint8_t)0xFA, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFA, (char)(uint8_t)0xFA, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0xFA, (char)(uint8_t)0xFA, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "TEMPERATURE") != -20 || getDoubleValue(peerId, 1, "HUMIDITY") != 100)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (2)" << std::endl;
			exit(1);
		}
	// }}}

	int32_t retries = 5;
	for(int32_t i = 250; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)i, (char)(uint8_t)i, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t temperatureValue = std::lround((getDoubleValue(peerId, 1, "TEMPERATURE") + 20) * 3.125);
		int32_t humidityValue = std::lround(getDoubleValue(peerId, 1, "HUMIDITY") * 2.5);
		if(temperatureValue != i || humidityValue != i)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << temperatureValue << ", " << humidityValue << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}
	deleteDevice(peerId);
}

void testA50403()
{
	std::cout << std::endl << "Testing EEP A50403... Values should go from 0% to 100% and from -20°C to 60°C... " << std::endl;
	uint64_t peerId = createDevice("A50403");

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, (char)(uint8_t)0, (char)(uint8_t)0x0, 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, 0x03, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, 0x03, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, 0x03, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "TEMPERATURE") != -20 || getDoubleValue(peerId, 1, "HUMIDITY") != 0)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	int32_t retries = 5;
	for(int32_t i = 1023; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)(i / 4), (char)(uint8_t)(i >> 8), (char)(uint8_t)(i & 0xFF), 0x0A, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t temperatureValue = std::lround((getDoubleValue(peerId, 1, "TEMPERATURE") + 20) * 12.7875);
		int32_t humidityValue = std::lround(getDoubleValue(peerId, 1, "HUMIDITY") * 2.55);
		if((temperatureValue != i && temperatureValue != i - 1  && temperatureValue != i + 1) || humidityValue != (i / 4))
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << temperatureValue << ", " << humidityValue << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}
	deleteDevice(peerId);
}

void testA50501()
{
	std::cout << std::endl << "Testing EEP A50501... Values should go from 500 hPa to 1150 hPa... " << std::endl;
	uint64_t peerId = createDevice("A50501");

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0x03, (char)(uint8_t)0xFF, 0, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0x03, (char)(uint8_t)0xFF, 0, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0x03, (char)(uint8_t)0xFF, 0, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "PRESSURE") != 500)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	int32_t retries = 5;
	for(int32_t i = 1023; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)(i >> 8), (char)(uint8_t)(i & 0xFF), 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value = std::lround((getDoubleValue(peerId, 1, "PRESSURE") -500) * 1.573846);
		if(value != i && value != i - 1  && value != i + 1)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong value returned for binary value " << i << ": " << value << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}
	deleteDevice(peerId);
}

void testA50601()
{
	std::cout << std::endl << "Testing EEP A50601... Values should go from 300 lx to 30000 lx for ILLUMINATION2 and from 600 lx to 60000 lx for ILLUMINATION1... " << std::endl;
	uint64_t peerId = createDevice("A50601");

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") != 0.0 || getIntValue(peerId, 1, "ILLUMINATION_1") != 600.0 || getIntValue(peerId, 1, "ILLUMINATION_2") != 300.0)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	int32_t retries = 5;
	for(int32_t i = 255; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)i, (char)(uint8_t)i, (char)(uint8_t)i, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value1 = std::lround(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") * 50.0);
		int32_t value2 = std::lround((getIntValue(peerId, 1, "ILLUMINATION_2") -300) * 0.0085858585);
		int32_t value3 = std::lround((getIntValue(peerId, 1, "ILLUMINATION_1") -600) * 0.0042929293);
		if(value1 != i || (value3 != i && value3 != i - 1  && value3 != i + 1) || value2 != 0)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << value1 << ' ' << value2 << ' ' << value3 << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}

	retries = 5;
	for(int32_t i = 255; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)i, (char)(uint8_t)i, (char)(uint8_t)i, 0x09, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value1 = std::lround(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") * 50.0);
		int32_t value2 = std::lround((getIntValue(peerId, 1, "ILLUMINATION_2") -300) * 0.0085858585);
		int32_t value3 = std::lround((getIntValue(peerId, 1, "ILLUMINATION_1") -600) * 0.0042929293);
		if(value1 != i || (value2 != i && value2 != i - 1  && value2 != i + 1) || value3 != 0)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << value1 << ' ' << value2 << ' ' << value3 << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}
	deleteDevice(peerId);
}

void testA50602()
{
	std::cout << std::endl << "Testing EEP A50602... Values should go from 0 lx to 510 lx for ILLUMINATION2 and from 0 lx to 1020 lx for ILLUMINATION1... " << std::endl;
	uint64_t peerId = createDevice("A50602");

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") != 0.0 || getIntValue(peerId, 1, "ILLUMINATION_1") != 0.0 || getIntValue(peerId, 1, "ILLUMINATION_2") != 0.0)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	int32_t retries = 5;
	for(int32_t i = 255; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)i, (char)(uint8_t)i, (char)(uint8_t)i, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value1 = std::lround(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") * 50.0);
		int32_t value2 = std::lround(getIntValue(peerId, 1, "ILLUMINATION_2") * 0.5);
		int32_t value3 = std::lround(getIntValue(peerId, 1, "ILLUMINATION_1") * 0.25);
		if(value1 != i || (value3 != i && value3 != i - 1  && value3 != i + 1) || value2 != 0)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << value1 << ' ' << value2 << ' ' << value3 << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}

	retries = 5;
	for(int32_t i = 255; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)i, (char)(uint8_t)i, (char)(uint8_t)i, 0x09, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value1 = std::lround(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") * 50.0);
		int32_t value2 = std::lround(getIntValue(peerId, 1, "ILLUMINATION_2") * 0.5);
		int32_t value3 = std::lround(getIntValue(peerId, 1, "ILLUMINATION_1") * 0.25);
		if(value1 != i || (value2 != i && value2 != i - 1  && value2 != i + 1) || value3 != 0)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << value1 << ' ' << value2 << ' ' << value3 << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}
	deleteDevice(peerId);
}

void testA50603()
{
	std::cout << std::endl << "Testing EEP A50603... Values should go from 0 lx to 1000 lx... " << std::endl;
	uint64_t peerId = createDevice("A50603");

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xC0, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xC0, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xC0, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") != 0.0 || getIntValue(peerId, 1, "ILLUMINATION") != 0)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	int32_t retries = 5;
	for(int32_t i = 1000; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)(i / 4), (char)(uint8_t)(i >> 2), (char)(uint8_t)((i & 0x3) << 6), 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value1 = std::lround(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") * 50.0);
		int32_t value2 = getIntValue(peerId, 1, "ILLUMINATION");
		if(value1 != (i / 4) || value2 != i)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << value1 << ' ' << value2 << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}

	deleteDevice(peerId);
}

void testA50604()
{
	std::cout << std::endl << "Testing EEP A50604... Values should go from 0 lx to 65535 lx and -20 °C to 60 °C... " << std::endl;
	uint64_t peerId = createDevice("A50604");

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x0B, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x0B, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x0B, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xF3, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xF3, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xF3, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "TEMPERATURE") != -20.0 || getIntValue(peerId, 1, "ILLUMINATION") != 0 || getIntValue(peerId, 1, "ENERGY_STORAGE") != 0)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	int32_t retries = 5;
	for(int32_t i = 1023; i >= 0; i--)
	{
		int32_t illuminance = std::lround(i * 64.06158357);
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)(i / 4), (char)(uint8_t)(illuminance >> 8), (char)(uint8_t)(illuminance & 0xFF), (char)(uint8_t)(((i % 16) << 4) | 0x0B), _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value1 = std::lround((getDoubleValue(peerId, 1, "TEMPERATURE") + 20.0) * 3.125);
		int32_t value2 = getIntValue(peerId, 1, "ILLUMINATION");
		int32_t value3 = std::lround(getIntValue(peerId, 1, "ENERGY_STORAGE") * 0.15);
		if(value1 != (i / 4) || value2 != illuminance || value3 != (i % 16))
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << value1 << ' ' << value2 << ' ' << value3 << ". Expected: " << (i / 4) << ' ' << illuminance << ' ' << (i % 16) << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}

	deleteDevice(peerId);
}

void testA50605()
{
	std::cout << std::endl << "Testing EEP A50605... Values should go from 0 lx to 5100 lx for ILLUMINATION2 and from 0 lx to 10200 lx for ILLUMINATION1... " << std::endl;
	uint64_t peerId = createDevice("A50605");

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") != 0.0 || getIntValue(peerId, 1, "ILLUMINATION_1") != 0.0 || getIntValue(peerId, 1, "ILLUMINATION_2") != 0.0)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	int32_t retries = 5;
	for(int32_t i = 255; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)i, (char)(uint8_t)i, (char)(uint8_t)i, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value1 = std::lround(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") * 50.0);
		int32_t value2 = std::lround(getIntValue(peerId, 1, "ILLUMINATION_2") * 0.05);
		int32_t value3 = std::lround(getIntValue(peerId, 1, "ILLUMINATION_1") * 0.025);
		if(value1 != i || (value3 != i && value3 != i - 1  && value3 != i + 1) || value2 != 0)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << value1 << ' ' << value2 << ' ' << value3 << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}

	retries = 5;
	for(int32_t i = 255; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)i, (char)(uint8_t)i, (char)(uint8_t)i, 0x09, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value1 = std::lround(getDoubleValue(peerId, 1, "SUPPLY_VOLTAGE") * 50.0);
		int32_t value2 = std::lround(getIntValue(peerId, 1, "ILLUMINATION_2") * 0.05);
		int32_t value3 = std::lround(getIntValue(peerId, 1, "ILLUMINATION_1") * 0.025);
		if(value1 != i || (value2 != i && value2 != i - 1  && value2 != i + 1) || value3 != 0)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong values returned for binary value " << i << ": " << value1 << ' ' << value2 << ' ' << value3 << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}
	deleteDevice(peerId);
}

void testA53801()
{
	std::cout << std::endl << "Testing EEP A53801... " << std::endl;
	uint64_t peerId = createDevice("A53801");

	setValue(peerId, 1, "PAIRING", 2);

	int64_t startTime = BaseLib::HelperFunctions::getTime();
	std::vector<char> packet;

	while(BaseLib::HelperFunctions::getTime() < startTime + 100)
	{
		readPacket(); // Clear buffer
	}

	startTime = BaseLib::HelperFunctions::getTime();
	setValue(peerId, 1, "STATE", true);
	while(packet.size() != 24 && BaseLib::HelperFunctions::getTime() < startTime + 2000)
	{
		packet = readPacket();
	}
	if(packet.empty() || packet.at(10) != 9 || (packet.at(14) & 0x7F) != 2)
	{
		std::cerr << "Wrong value received for value \"true\": " << BaseLib::HelperFunctions::getHexString(packet) << std::endl;
		deleteDevice(peerId);
		exit(1);
	}

	packet.clear();
	startTime = BaseLib::HelperFunctions::getTime();
	setValue(peerId, 1, "STATE", false);
	while(packet.size() != 24 && BaseLib::HelperFunctions::getTime() < startTime + 2000)
	{
		packet = readPacket();
	}
	if(packet.empty() || packet.at(10) != 8 || (packet.at(14) & 0x7F) != 2)
	{
		std::cerr << "Wrong value received for value \"false\": " << BaseLib::HelperFunctions::getHexString(packet) << std::endl;
		deleteDevice(peerId);
		exit(1);
	}

	packet.clear();
	startTime = BaseLib::HelperFunctions::getTime();
	setValue(peerId, 1, "STATE", true);
	while(packet.size() != 24 && BaseLib::HelperFunctions::getTime() < startTime + 2000)
	{
		packet = readPacket();
	}
	if(packet.empty() || packet.at(10) != 9 || (packet.at(14) & 0x7F) != 2)
	{
		std::cerr << "Wrong value received for value \"true\": " << BaseLib::HelperFunctions::getHexString(packet) << std::endl;
		deleteDevice(peerId);
		exit(1);
	}

	deleteDevice(peerId);
}

void testA53802()
{
	std::cout << std::endl << "Testing EEP A53802... " << std::endl;
	uint64_t peerId = createDevice("A53802");

	setValue(peerId, 1, "PAIRING", 2);

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 2, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 2, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 2, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 2, (char)(uint8_t)0xFF, 0, 1, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 2, (char)(uint8_t)0xFF, 0, 1, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 2, (char)(uint8_t)0xFF, 0, 1, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getIntValue(peerId, 1, "LEVEL") != 0)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	int64_t startTime = BaseLib::HelperFunctions::getTime();
	std::vector<char> packet;

	while(BaseLib::HelperFunctions::getTime() < startTime + 100)
	{
		readPacket(); // Clear buffer
	}

	packet.clear();
	startTime = BaseLib::HelperFunctions::getTime();
	setValue(peerId, 1, "LEVEL", 0);
	while(packet.size() != 24 && BaseLib::HelperFunctions::getTime() < startTime + 2000)
	{
		packet = readPacket();
	}
	if(packet.empty() || packet.at(10) != 8 || (packet.at(14) & 0x7F) != 2)
	{
		std::cerr << "Wrong value received for value \"0\": " << BaseLib::HelperFunctions::getHexString(packet) << std::endl;
		deleteDevice(peerId);
		exit(1);
	}

	for(int32_t i = 2; i <= 255; i++)
	{
		packet.clear();
		startTime = BaseLib::HelperFunctions::getTime();
		setValue(peerId, 1, "RAMPING_TIME", i);
		setValue(peerId, 1, "LEVEL", (int32_t)std::lround(i / 2.55));
		while(packet.size() != 24 && BaseLib::HelperFunctions::getTime() < startTime + 2000)
		{
			packet = readPacket();
		}
		if(packet.empty() || packet.at(8) != (char)(uint8_t)std::lround(std::lround(i / 2.55) * 2.55) || packet.at(9) != (char)(uint8_t)i || packet.at(10) != 9 || (packet.at(14) & 0x7F) != 2)
		{
			std::cerr << "Wrong value received for value \"" << i << "\" (expected \"0x" << std::hex << std::lround(std::lround(i / 2.55) * 2.55) << "\"): " << BaseLib::HelperFunctions::getHexString(packet) << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
	}

	deleteDevice(peerId);
}

void testA5()
{
	/*testA502("A50201", 255, 0, 6.375);
	testA502("A50202", 255, 10, 6.375);
	testA502("A50203", 255, 20, 6.375);
	testA502("A50204", 255, 30, 6.375);
	testA502("A50205", 255, 40, 6.375);
	testA502("A50206", 255, 50, 6.375);
	testA502("A50207", 255, 60, 6.375);
	testA502("A50208", 255, 70, 6.375);
	testA502("A50209", 255, 80, 6.375);
	testA502("A5020A", 255, 90, 6.375);
	testA502("A5020B", 255, 100, 6.375);
	testA502("A50210", 255, 20, 3.1875);
	testA502("A50211", 255, 30, 3.1875);
	testA502("A50212", 255, 40, 3.1875);
	testA502("A50213", 255, 50, 3.1875);
	testA502("A50214", 255, 60, 3.1875);
	testA502("A50215", 255, 70, 3.1875);
	testA502("A50216", 255, 80, 3.1875);
	testA502("A50217", 255, 90, 3.1875);
	testA502("A50218", 255, 100, 3.1875);
	testA502("A50219", 255, 110, 3.1875);
	testA502("A5021A", 255, 120, 3.1875);
	testA502("A5021B", 255, 130, 3.1875);
	testA502("A50220", 1023, 41.2, 20);
	testA502("A50230", 1023, 62.3, 10);
	testA50401();
	testA50402();
	testA50403();
	testA50501();
	testA50601();
	testA50602();*/
	testA50603();
	testA50604();
	testA50605();
	/*testA53801();
	testA53802();*/
}

void testF60201()
{
	std::cout << std::endl << "Testing EEP F60201... " << std::endl;
	uint64_t peerId = createDevice("F60201");

	// {{{ LRN bit
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0, 0, 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0x03, (char)(uint8_t)0xFF, 0, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0x03, (char)(uint8_t)0xFF, 0, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, 0x03, (char)(uint8_t)0xFF, 0, 0, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		if(getDoubleValue(peerId, 1, "PRESSURE") != 500)
		{
			deleteDevice(peerId);
			std::cerr << "Wrong value returned (1)" << std::endl;
			exit(1);
		}
	// }}}

	int32_t retries = 5;
	for(int32_t i = 1023; i >= 0; i--)
	{
		if(retries != 5) i++;
		sendPacket(std::vector<char>{ 0x55, 0x00, 0x0A, 0x07, 0x01, 0x00, (char)(uint8_t)0xA5, (char)(uint8_t)(i >> 8), (char)(uint8_t)(i & 0xFF), 0, 0x08, _byteAddress[0], _byteAddress[1], _byteAddress[2], _byteAddress[3], 0x00, 0x01, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, (char)(uint8_t)0xFF, 0x00, 0x00, 0x00 });
		int32_t value = std::lround((getDoubleValue(peerId, 1, "PRESSURE") -500) * 1.573846);
		if(value != i && value != i - 1  && value != i + 1)
		{
			retries--;
			if(retries > 0)
			{
				std::cout << 'r';
				continue;
			}
			std::cerr << "Wrong value returned for binary value " << i << ": " << value << std::endl;
			deleteDevice(peerId);
			exit(1);
		}
		retries = 5;
		std::cout << (i > 0 ? "." : ".; done.\n") << std::flush;
	}
	deleteDevice(peerId);
}

void testF6()
{
	//testF60201();
}