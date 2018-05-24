/*
    Embeded Software Systems Project 2018, TGGS, KMUTNB   
    Platform Robot Configuration files for Meccanum Platform Robot, 
    Author: Mix, Bekty
*/
#include <Motor.h>
#include <SONAR.h>
#include <Wire.h>

#define compass_addr 0x60 //defines address of compass

SONAR s11 = SONAR(0x11); //defines addresses of SONAR sensor
SONAR s12 = SONAR(0x12);
SONAR s13 = SONAR(0x13);
SONAR s14 = SONAR(0x14);

static unsigned char crctable[256];
uint8_t data[50];
uint8_t data_correct[11];
int ind_data = 0;
bool dataready = false;
float comps_yaw = 0;
int distance[4] = {0, 0, 0, 0};
bool obstacle[4] = {0, 0, 0, 0};

enum
{
    S_LEFT = 0,
    S_FRONT = 1,
    S_RIGHT = 2,
    S_REAR = 3,
    M_FL = 0,
    M_FR = 1,
    M_RL = 2,
    M_RR = 3,
};

Motor motorFL(23, 2, 3, 6, 7, false);
Motor motorFR(21, 16, 17, 25, 26, false);
Motor motorRL(22, 5, 4, 8, 11, false);
Motor motorRR(20, 14, 15, 28, 27, false);

/* External Interrupt for wheel FL routine*/
void isr_ext_1()
{
    noInterrupts();
    motorFL.count();
    interrupts();
}

/* External Interrupt for wheel FR routine*/
void isr_ext_2()
{
    noInterrupts();
    motorFR.count();
    interrupts();
}

/* External Interrupt for wheel RL routine*/
void isr_ext_3()
{
    noInterrupts();
    motorRL.count();
    interrupts();
}

/* External Interrupt for wheel RR routine*/
void isr_ext_4()
{
    noInterrupts();
    motorRR.count();
    interrupts();
}

/* Motor setup*/
void motion_init()
{
    motorFL.init();
    motorFR.init();
    motorRL.init();
    motorRR.init();
    attachInterrupt(digitalPinToInterrupt(motorFL.pinEncA), isr_ext_1, RISING);
    attachInterrupt(digitalPinToInterrupt(motorFR.pinEncA), isr_ext_2, RISING);
    attachInterrupt(digitalPinToInterrupt(motorRL.pinEncA), isr_ext_3, RISING);
    attachInterrupt(digitalPinToInterrupt(motorRR.pinEncA), isr_ext_4, RISING);
}

/* CMPS03 Compass I2C data polling*/
void getYaw()
{
    byte highByte;
    byte lowByte;
    Wire.beginTransmission(compass_addr); //starts communication with cmps03
    Wire.write(2);                        //Sends the register we wish to read
    Wire.endTransmission();
    Wire.requestFrom(compass_addr, 2); //requests high byte
    while (Wire.available() < 2)
        ;                   //while there is a byte to receive
    highByte = Wire.read(); //reads the byte as an integer
    lowByte = Wire.read();
    int bearing = ((highByte << 8) + lowByte);
    comps_yaw = ((float)bearing) / 10.0;
}

/* RS485 SONAR serial data polling*/
void getDistance()
{
    s11.trigger(); // 3ms
    s12.trigger();
    s13.trigger();
    s14.trigger();
    distance[S_LEFT] = s11.getDist(); // 10ms
    distance[S_FRONT] = s12.getDist();
    distance[S_RIGHT] = s13.getDist();
    distance[S_REAR] = s14.getDist();

    for (int i = 0; i < 4; i++)
    {
        if (distance[i] < 20)
            obstacle[i] = 1;
        else
            obstacle[i] = 0;
    }
}

void getSpeed()
{
    motorFL.computeSpeed();
    motorFR.computeSpeed();
    motorRL.computeSpeed();
    motorRR.computeSpeed();
}

/* Computing CRC8 checksum*/
unsigned char compute_crc8(unsigned char *bytes, unsigned int length)
{
    unsigned char crc = 0;
    for (int i = 0; i < length; i++)
    {
        /* XOR-in next input byte */
        unsigned char data = (unsigned char)(bytes[i] ^ crc);
        /* get current CRC value = remainder */
        crc = (unsigned char)(crctable[data]);
    }
    return crc;
}

static void calulatetable_crc8()
{
    const unsigned char generator = 0x1D;
    /* iterate over all byte values 0 - 255 */
    for (int divident = 0; divident < 256; divident++)
    {
        unsigned char currByte = (unsigned char)divident;
        /* calculate the CRC-8 value for current byte */
        for (unsigned char bit = 0; bit < 8; bit++)
        {
            if ((currByte & 0x80) != 0)
            {
                currByte <<= 1;
                currByte ^= generator;
            }
            else
            {
                currByte <<= 1;
            }
        }
        /* store CRC value in lookup table */
        crctable[divident] = currByte;
    }
}
