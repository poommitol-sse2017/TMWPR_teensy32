#include <Motor.h>

Motor::Motor(unsigned char _pinPWM, unsigned char _pinDirA, unsigned char _pinDirB, unsigned char _pinEncA, unsigned char _pinEncB, bool _isInvert)
{
	pinPWM = _pinPWM;
	pinDirA = _pinDirA;
	pinDirB = _pinDirB;
	pinEncA = _pinEncA;
	pinEncB = _pinEncB;
	isInvert = _isInvert;
	curDir = 0;
	//lastDir = curDir;
}
void Motor::init()
{
	pinMode(pinPWM, OUTPUT);
	pinMode(pinDirA, OUTPUT);
	pinMode(pinDirB, OUTPUT);
	pinMode(pinEncA, INPUT_PULLUP);
	pinMode(pinEncB, INPUT_PULLUP);
	analogWriteFrequency(pinPWM, 31250);
}

float Motor::getSpeedRPM()
{
	//if (curDir == 0)
	return speedRPM;
	// else
	// 	return -speedRPM;
}

void Motor::setSpeedRPM(float _speedRPM)
{
	speedRPM = _speedRPM;
}

void Motor::count()
{
	//noInterrupts();
	if (digitalReadFast(pinEncB))
		enc_count++;
	else
		enc_count--;
	//interrupts();
}

void Motor::clear()
{
	enc_count = 0;
	tc = 0;
}

void Motor::computeSpeed(float _time)
{
	speedRPM = ((float(enc_count) / 768.0) / (float(_time) / 1000000)) * 60;
	enc_count = 0;
}
void Motor::computeSpeed()
{
	/*
		to reduce complexity, 
		1 rotation round for motor (64:1,12ppr rising triggered) is 768 pulse
		time is in microsecond, thus need to be divide by 1M
		as the speed is calculated on RPM, we need to multiply by 60
		from equation, speedRPM = (encoder_count/768)*(time)
	*/
	float times = (float(tc) / 78125);
	speedRPM = (float(enc_count) / times);
	enc_count = 0;
	tc = 0;
}
void Motor::setGain(float _KP, float _KI, float _KD)
{
	KP = _KP;
	KI = _KI;
	KD = _KD;
}

void Motor::runPWM(int PWM)
{
	if (isInvert)
		PWM = -PWM;

	if (PWM > 0)
	{
		digitalWrite(pinDirA, HIGH);
		digitalWrite(pinDirB, LOW);

		if (PWM > 255)
			PWM = 255;
		curDir = 0;
	}
	else if (PWM < 0)
	{
		digitalWrite(pinDirA, LOW);
		digitalWrite(pinDirB, HIGH);
		if (PWM < -255)
			PWM = -255;
		curDir = 1;
	}
	analogWrite(pinPWM, abs(PWM));
}

void Motor::runRPM(float targetRPM)
{
	error = targetRPM - speedRPM;

	integral = integral + (error * ((float)mtc / 1000000));
	if (abs(integral) > integral_sat)
	{
		if (integral < 0)
			integral = -integral_sat;
		else
			integral = integral_sat;
	}
	derivative = (error - last_error) / ((float)mtc / 1000000);
	last_error = error;

	output_pwm = (KP * error + KI * integral + KD * derivative);
	//Serial.println(u);
	cur_pwm = cur_pwm + output_pwm;

	// Serial.print("mtc ");
	// Serial.println(mtc);
	// Serial.print("error ");
	// Serial.println(error);
	// Serial.print("integral ");
	// Serial.println(integral);

	// Serial.print("derivative ");
	// Serial.println(derivative);

	// Serial.print("pwm ");
	// Serial.println(cur_pwm);
	mtc = 0;

	if (targetRPM == 0)
	{
		integral = 0;
		cur_pwm = 0;
		Motor::brake();
	}
	else
	{
		if (cur_pwm == 0)
		{
			Motor::runPWM(0);
			//Motor::drift();
		}
		else if (cur_pwm < -255)
		{
			cur_pwm = -255;
			Motor::runPWM(cur_pwm);
		}
		else if (cur_pwm > 255)
		{
			cur_pwm = 255;
			Motor::runPWM(cur_pwm);
		}
		else
		{
			Motor::runPWM(cur_pwm);
		}
	}
}
void Motor::brake()
{
	digitalWrite(pinDirA, LOW);
	digitalWrite(pinDirB, LOW);
	digitalWrite(pinPWM, HIGH);
}

void Motor::drift()
{
	digitalWrite(pinDirA, LOW);
	digitalWrite(pinDirB, LOW);
	digitalWrite(pinPWM, LOW);
}
