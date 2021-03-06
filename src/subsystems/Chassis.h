#ifndef Chassis_h
#define Chassis_h

#include <WPILib.h>

class Chassis: public Subsystem {
	public:
		Chassis();
		~Chassis();
		void InitDefaultCommand();
		void drive(double vX, double vY, double vR, double throttle, bool weBePimpin = false);
		
	private:
		Victor* driveMotorA;
		Victor* driveMotorB;
		Victor* driveMotorC;
		Victor* driveMotorD;
		void liveWindow();

		PIDController* pidA;
		PIDController* pidB;
		PIDController* pidC;
		PIDController* pidD;
		Encoder *encoderA, *encoderB, *encoderC, *encoderD;
		
		Gyro *gyro;
};

#endif
