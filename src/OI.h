#ifndef OI_H
#define OI_H

#include "WPILib.h"

class OI {
private:
	Joystick* joyDrv;
	Button* catchButton;
	Button* chopsticksSpinUpButton;
	Button* chopsticksSpinDownButton;
	Button* chopsticksMoveUpButton;
	Button* chopsticksMoveDownButton;
	Button* throwButton;
	double applyDeadZone(double,double);

public:
	OI();
	
	Joystick* getJoyDrv();
	double getJoyDrvX();
	double getJoyDrvY();
	double getJoyDrvZ();
	double getJoyDrvThrottle();
	bool weBePimpin();
};

#endif
