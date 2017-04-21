#include <homegear-base/BaseLib.h>
#include <string>
#include <iostream>
#include <vector>

int main(int argc, char* argv[])
{
	std::unique_ptr<BaseLib::SharedObjects> bl(new BaseLib::SharedObjects(std::string(""), nullptr, false));
	BaseLib::SerialReaderWriter serial(bl.get(), "/dev/ttyUSB0", 57600, 0, true, -1);
	serial.openDevice(false, false, false);

	int32_t size = 0;
	std::vector<char> dataArray;
	while(true)
	{
		char data;
		int32_t result = serial.readChar(data, 5000000);
		if(result == -1)
		{
			std::cerr << "Error" << std::endl;
			return -1;
		}
		else if(result == 1)
		{
			size = 0;
			dataArray.clear();
			continue;
		}

		if(dataArray.empty() && data != 0x55) continue;

		dataArray.push_back(data);
		if(size == 0 && dataArray.size() == 6) size = ((dataArray[1] << 8) | dataArray[2]) + dataArray[3] + 7;
		if(size > 0 && dataArray.size() == size)
		{
			size = 0;
			std::cout << BaseLib::HelperFunctions::getHexString(dataArray) << std::endl;
			dataArray.clear();
		}
	}

	serial.closeDevice();

	return 0;
}
