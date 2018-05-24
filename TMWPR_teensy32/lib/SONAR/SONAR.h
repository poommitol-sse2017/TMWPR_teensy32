
#ifndef SONAR_H
#define SONAR_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#define	SonarPrint	Serial2

class SONAR {
public:
	SONAR(unsigned char addr=0x11);
	unsigned char sendCmd(unsigned char* cmd,unsigned char size);
	unsigned char recvDat(unsigned char size=bufSize);
	char checksum(unsigned char size=bufSize);
	unsigned char showDat(unsigned char size=bufSize);
	unsigned char setAddr(unsigned char addr=0);
	static void init(unsigned char pinCtrl=13,unsigned int baudrate=baudRate);
	static void release();
	unsigned char trigger();
	unsigned int getDist();
	int getTemp();
	unsigned int update();
	unsigned char getAddr() const;
	unsigned char initAddr(unsigned char addr);
	enum {
			addrCmdSize=7,
			addrDatSize=7,
			trigCmdSize=6,
			distCmdSize=6,
			distDatSize=8,
			tempCmdSize=6,
			tempDatSize=8,
			bufSize=8,
			duration=60,
			baudRate=19200,
	};

private:

	static const unsigned char addrCmdTemplate[addrCmdSize];
	static const unsigned char trigCmdTemplate[trigCmdSize];
	static const unsigned char distCmdTemplate[distCmdSize];
	static const unsigned char tempCmdTemplate[tempCmdSize];
	unsigned char _addr;
	unsigned char _trigCmd[sizeof(trigCmdTemplate)];
	unsigned char _distCmd[sizeof(distCmdTemplate)];
	unsigned char _recvBuf[bufSize];

	static unsigned char* generateAddrCmd(unsigned char* addrCmd,unsigned char addr);
	static unsigned char* generateTempCmd(unsigned char* tempCmd,unsigned char addr);
	unsigned char* generateTrigCmd();
	unsigned char* generateDistCmd();
	unsigned char clearBuf();	
};

#endif




