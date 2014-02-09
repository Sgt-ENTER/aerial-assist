#include "MeccanumDrive.h"
#include "../Robotmap.h"

MeccanumDrive::MeccanumDrive():CommandBase("MeccanumDrive"){
	Requires(chassis);
}

void MeccanumDrive::Initialize(){	
}

void MeccanumDrive::Execute(){
    double x = oi->getJoyDrvX();
    double y = oi->getJoyDrvY();
    double z = oi->getJoyDrvZ();
    double throttle = oi->getJoyDrvThrottle();
    bool weBePimpin = oi->weBePimpin();
   
	chassis->drive(x, y, z, throttle,weBePimpin);
    
    // Put the values onto the SmartDashboard
    SmartDashboard::PutNumber("Joystick x", x);
    SmartDashboard::PutNumber("Joystick y", y);
    SmartDashboard::PutNumber("Joystick z", z);
    SmartDashboard::PutNumber("Joystick throttle", throttle);
}

bool MeccanumDrive::IsFinished(){
	return false;
}

void MeccanumDrive::End(){
}

void MeccanumDrive::Interrupted(){
	chassis->drive (0,0,0,0);
}	
