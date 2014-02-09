#include "OI.h"
#include "Robotmap.h"
#include "commands/CatchBall.h"
#include "commands/ChopsticksPickUp.h"
#include "commands/ChopsticksPutDown.h"
#include "commands/ThrowBall.h"

OI::OI() {
	// Process operator interface input here.
	joyDrv = new Joystick(1);
	
	catchButton = new JoystickButton(joyDrv,CATCH_BUTTON);
	catchButton->WhileHeld(new CatchBall());
	
	throwButton = new JoystickButton(joyDrv,THROW_BUTTON);
	throwButton->WhenPressed(new ThrowBall());
	
	chopsticksUpButton = new JoystickButton(joyDrv,CHOPSTICKS_UP_BUTTON);
	chopsticksUpButton->WhileHeld(new ChopsticksPickUp());
	chopsticksDownButton = new JoystickButton(joyDrv,CHOPSTICKS_DOWN_BUTTON);
	chopsticksDownButton->WhileHeld(new ChopsticksPutDown());
}

Joystick* OI::getJoyDrv() {
	return joyDrv;
}

double OI :: applyDeadZone(double input, double deadZone){
	if(input <= - deadZone || input >= deadZone){
		return input;
	}
	return 0.0;
}

double OI::getJoyDrvX(){
	return -applyDeadZone(joyDrv->GetY(), JOY_DRV_DEAD_X);
}

double OI::getJoyDrvY(){
	return -applyDeadZone(joyDrv->GetX(), JOY_DRV_DEAD_Y);
}

double OI::getJoyDrvZ(){
	return -applyDeadZone(joyDrv->GetZ(), JOY_DRV_DEAD_Z);
}

double OI::getJoyDrvThrottle(){
	return -(joyDrv->GetTwist()-1.0)/2.0;
}

bool OI::weBePimpin(){
	return joyDrv->GetRawButton(PIMP_ROLL_BUTTON);
}
