#include "Arduino.h"
#include "MOSFETMotor.h"
#include <pwm.h>



MOSFETMotor::MOSFETMotor(int Channel1, int Channel2, int PowerChannel)
{
	pinMode(Channel1, OUTPUT);
	pinMode(Channel2, OUTPUT);
	pinMode(PowerChannel, OUTPUT);
	A = Channel1;
	B = Channel2;
	P = PowerChannel;
	PowerDOWN();
	Brake();
	PowerStatus = false;
	BridgeMode = "brake";
	
}



void MOSFETMotor::PowerDOWN()
{
	digitalWrite(P, LOW);
	PowerStatus = false;
}

void MOSFETMotor::PowerUP()
{
	digitalWrite(P, HIGH);
	PowerStatus = true;
}

void MOSFETMotor::Forward()
{
	if (BridgeMode != "forward")
	{

		if (PowerStatus == true)
		{
			PowerDOWN();
			delayMicroseconds(1);
		}

		digitalWrite(A, HIGH);
		digitalWrite(B, LOW);
		
		BridgeMode = "forward";
		delayMicroseconds(1);
		PowerUP();

	}

}

void MOSFETMotor::Backward()
{
	if (BridgeMode != "backward")
	{

		if (PowerStatus == true)
		{
			PowerDOWN();
			delayMicroseconds(1);
		}

		digitalWrite(A, LOW);
		digitalWrite(B, HIGH);
		
		BridgeMode = "backward";
		delayMicroseconds(1);
		PowerUP();

	}

}

void MOSFETMotor::Brake()
{
	if (BridgeMode != "brake")
	{
		if (PowerStatus == true)
		{
			PowerDOWN();
			delayMicroseconds(1);
		}

		digitalWrite(A, LOW);
		digitalWrite(B, LOW);

		BridgeMode = "brake";
		delayMicroseconds(1);
		PowerUP();
	}
	
}

void MOSFETMotor::SetPWM(uint16 pwm)
{
	if (pwm >= 0 && pwm <= 255) analogWrite(P, pwm);
}
