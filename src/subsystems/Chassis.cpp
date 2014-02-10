#include <math.h>

#include "Chassis.h"
#include "../Robotmap.h"
#include "../commands/MeccanumDrive.h"

Chassis::Chassis():Subsystem("Chassis"),gyro(new Gyro(GYRO_PORT)){
    driveMotorA = new Victor(MOTOR_A_PWM);
    driveMotorB = new Victor(MOTOR_B_PWM);
    driveMotorC = new Victor(MOTOR_C_PWM);
    driveMotorD = new Victor(MOTOR_D_PWM);
    
    //we are waiting for the gyro to stabilize
    Wait(2.0);
    gyro->Reset();
    
}

Chassis::~Chassis() {
	delete driveMotorA;
	delete driveMotorB;
	delete driveMotorC;
	delete driveMotorD;
	delete gyro;
}

void Chassis::drive(double vX, double vY, double vZ, double throttle, bool weBePimpin) {
	double vMotor[4];
	
	//this maps the body co-ordinates to the absolute field co-ordinates
	if(weBePimpin){
		double heading = gyro->GetAngle()*3.14159/180.0;
		double vXpimp = vX*cos(heading)+vY*sin(heading);
		double vYpimp = -vX*sin(heading)+vY*cos(heading);
		vX = vXpimp;
		vY = vYpimp;
	}
	
	vMotor[0] = vX - vY - vZ;
	vMotor[1] = vX + vY - vZ;
	vMotor[2] = -vX + vY - vZ;
	vMotor[3] = -vX - vY - vZ;
	
	double vmax = 1.0;
	for(int i = 0; i < 4; ++i){
		if(abs(vMotor[i]) >vmax ) {
			vmax =abs (vMotor [i]);
		}
	}
	for (int i = 0; i < 4; ++i){
		vMotor[i] = vMotor[i]/vmax*throttle; 
	}
	
	
	driveMotorA->Set(vMotor[0]);
    driveMotorB->Set(vMotor[1]);
    driveMotorC->Set(vMotor[2]);
    driveMotorD->Set(vMotor[3]);
    
    // Put the values onto the SmartDashboard
    SmartDashboard::PutNumber("Motor A", vMotor[0]);
    SmartDashboard::PutNumber("Motor B", vMotor[1]);
    SmartDashboard::PutNumber("Motor C", vMotor[2]);
    SmartDashboard::PutNumber("Motor D", vMotor[3]);
    SmartDashboard::PutNumber("Gyro(deg)", gyro->GetAngle());

}

void Chassis::InitDefaultCommand() {
    SetDefaultCommand(new MeccanumDrive());
}
    

