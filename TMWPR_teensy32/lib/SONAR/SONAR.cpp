#include <SONAR.h>
//static vars
const unsigned char SONAR::addrCmdTemplate[] = {0x55, 0xaa, 0xab, 0x01, 0x55, 0xff, 0x00};
// const unsigned char SONAR::addrDatTemplate[]={0x55,0xaa,0xff,0x01,0x55,0xff,0xff};
const unsigned char SONAR::trigCmdTemplate[] = {0x55, 0xaa, 0xff, 0x00, 0x01, 0x00};
const unsigned char SONAR::distCmdTemplate[] = {0x55, 0xaa, 0xff, 0x00, 0x02, 0x00};
//const unsigned char SONAR::distDattemplate[]={0x55,0xaa,0xff,0x02,0x02,0xff,0xff,0xff};
const unsigned char SONAR::tempCmdTemplate[] = {0x55, 0xaa, 0xff, 0x00, 0x03, 0x00};
//const unsigned char SONAR::tempDatTemplate[]={0x55,0xaa,0xff,0x02,0x03,0xff,0xff,0xff};


SONAR::SONAR(unsigned char addr)
{

	initAddr(addr);
}

unsigned char SONAR::initAddr(unsigned char addr)
{

	if (0x11 <= addr && addr <= 0x30)
		_addr = addr;
	generateTrigCmd();
	generateDistCmd();
	return getAddr();
}

unsigned char SONAR::getAddr() const
{

	return _addr;
}

unsigned char SONAR::setAddr(unsigned char addr)
{
	unsigned char addrCmd[sizeof(addrCmdTemplate)];
	if (0x11 <= addr && addr <= 0x30)
	{
		generateAddrCmd(addrCmd, addr);
		sendCmd(addrCmd, sizeof(addrCmd));
		delay(1);
		recvDat(addrDatSize);
		if (checksum(addrDatSize) == 0)
		{
			initAddr(addr);
			return addr;
		}
	}
	return 0;
}

unsigned char *SONAR::generateAddrCmd(unsigned char *addrCmd, unsigned char addr)
{

	for (int i = 0; i < sizeof(addrCmdTemplate); ++i)
	{
		addrCmd[i] = addrCmdTemplate[i];
	}
	addrCmd[sizeof(addrCmdTemplate) - 2] = addr; // addr
	for (int i = 0; i < sizeof(addrCmdTemplate) - 1; ++i)
	{ // checksum
		addrCmd[sizeof(addrCmdTemplate) - 1] += addrCmd[i];
	}
	return addrCmd;
}
unsigned char *SONAR::generateTrigCmd()
{

	for (int i = 0; i < sizeof(trigCmdTemplate); ++i)
	{
		_trigCmd[i] = trigCmdTemplate[i];
	}
	_trigCmd[2] = getAddr();
	for (int i = 0; i < sizeof(trigCmdTemplate) - 1; ++i)
	{
		_trigCmd[sizeof(trigCmdTemplate) - 1] += _trigCmd[i];
	}
	return _trigCmd;
}
unsigned char *SONAR::generateDistCmd()
{

	for (int i = 0; i < sizeof(distCmdTemplate); ++i)
	{
		_distCmd[i] = distCmdTemplate[i];
	}
	_distCmd[2] = getAddr();
	for (int i = 0; i < sizeof(distCmdTemplate) - 1; ++i)
	{
		_distCmd[sizeof(distCmdTemplate) - 1] += _distCmd[i];
	}
	return _distCmd;
}
unsigned char *SONAR::generateTempCmd(unsigned char *tempCmd, unsigned char addr)
{

	for (int i = 0; i < sizeof(tempCmdTemplate); ++i)
	{
		tempCmd[i] = tempCmdTemplate[i];
	}
	tempCmd[2] = addr;
	for (int i = 0; i < sizeof(tempCmdTemplate) - 1; ++i)
	{
		tempCmd[sizeof(tempCmdTemplate) - 1] += tempCmd[i];
	}
	return tempCmd;
}

unsigned char SONAR::sendCmd(unsigned char *cmd, unsigned char size)
{

	init(); // 201209
	////setTx();
	for (int i = 0; i < size; ++i)
	{
		//Serial2.print(cmd[i]);
		Serial2.write(cmd[i]); // 201204
	}
	Serial2.flush(); // 201204
	return size;
}
unsigned char SONAR::clearBuf()
{

	for (int i = 0; i < sizeof(_recvBuf); ++i)
	{
		_recvBuf[i] = 0;
	}
	return sizeof(_recvBuf);
}
unsigned char SONAR::recvDat(unsigned char desiredSize)
{

	clearBuf();
	init(); // 201209
	//setRX();
	unsigned char datSize = 0;

	for (int j = 0; datSize < desiredSize && j < 5000; ++j)
	{
		unsigned char ibyte = Serial2.read();
		//Serial.println(ibyte,HEX);
		delayMicroseconds(1);
		if (ibyte != 0xff)
		{
			_recvBuf[datSize++] = ibyte;
		}
	}
	delay(1);

	//Serial2.println(Serial2.read(), HEX);
	////setTx();
	
	return datSize;
}
char SONAR::checksum(unsigned char desiredSize)
{
	if (_recvBuf[0] == 0)
		return -1;
	unsigned char sum = 0;
	for (int i = 0; i < desiredSize - 1; ++i)
	{
		sum += _recvBuf[i];
	}
	if (sum != _recvBuf[desiredSize - 1])
		return -1;
	//if(_recvBuf[2]!=getAddr()) return -1; //
	return 0;
}
unsigned char SONAR::showDat(unsigned char desiredSize)
{

	////setTx();
	for (int i = 0; i < desiredSize; ++i)
	{
		Serial.print(_recvBuf[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
	return desiredSize;
}
void SONAR::init(unsigned char pinCtrl, unsigned int baudrate)
{
	Serial2.begin(baudrate);
}
void SONAR::release()
{
	Serial2.end();
}
unsigned char SONAR::trigger()
{

	return sendCmd(_trigCmd, sizeof(_trigCmd));
}
unsigned int SONAR::getDist()
{
	sendCmd(_distCmd, sizeof(_distCmd));
	delay(1);
	recvDat(distDatSize);
#ifdef DEBUG
	showDat(distDatSize);
#endif

	if (checksum(distDatSize) == 0)
		return (_recvBuf[5] << 8) + _recvBuf[6];

	return 0xffff; // not available distance
}
int SONAR::getTemp()
{
	unsigned char tempCmd[sizeof(tempCmdTemplate)];
	generateTempCmd(tempCmd, getAddr());
	sendCmd(tempCmd, sizeof(tempCmd));
	delay(1);
	recvDat(tempDatSize);

	if (checksum(tempDatSize) == 0)
	{
		int temp = (((_recvBuf[5] & 0x0f) << 8) + _recvBuf[6]) / 10;
		if ((_recvBuf[5] & 0xf0) == 0)
			return temp;
		return -1 * temp;
	}
	return 0xffff;
}

