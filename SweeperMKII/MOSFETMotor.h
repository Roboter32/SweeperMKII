#ifndef MOSFETMotor_h
#define MOSFETMotor_h

#include "Arduino.h"
#include <pwm.h>



class MOSFETMotor
{
public:
	MOSFETMotor(int Channel1, int Channel2, int PowerChannel);

	void Forward(); 
	void Backward(); //H-bridge modes
	void Brake();

	void PowerUP();
	void PowerDOWN(); //Power channel features
	void SetPWM(uint16 pwm);

private:
	int A, B, P;
	bool PowerStatus;
	String BridgeMode;

};

#endif MOSFETMotor_h
