/*
    Embeded Software Systems Project 2018, TGGS, KMUTNB
    
    Motor Control Library for Meccanum Platform Robot,
    
    Auther, Mix, Bekty
*/

#ifndef Motor_H
#define Motor_H
#include "Arduino.h"


class Motor
{
public:
	Motor(unsigned char _pinPWM, unsigned char _pinDirA,unsigned char _pinDirB, unsigned char _pinEncA,unsigned char _pinEnc, bool _isInvert);
	void init();
	void runPWM(int PWM);
	void runRPM(float targetRPM);
	float getSpeedRPM();
	void setSpeedRPM(float _speedRPM);
	void count();
	void clear();
	void computeSpeed(float _time);
	void computeSpeed();
	void brake();
	void drift();
	void setGain(float _KP,float _KI,float _KD);
	unsigned char pinEncA; //Because we can't put attachedInterrupted into class.
	long enc_count = 0;
	
private:
	bool isInvert = false;
	unsigned char pinPWM;
	unsigned char pinDirA;
	unsigned char pinDirB;
	unsigned char pinEncB;
	unsigned int curDir;
	float error=0;
	float last_error=0;
	float integral=0;
	float integral_sat=255;
    float derivative=0;
	float cur_pwm;
	float output_pwm;
	
	float KP, KI, KD;
	//unsigned int lastDir;
	elapsedMicros tc;
	elapsedMicros mtc;
	
	float speedRPM; // current PWM
	
	
};

#endif