
#include "RunTracking.h"
#include "../Robotmap.h"

RunTracking::RunTracking():CommandBase("RunTracking"){
	Requires(beaglebone);
}

void RunTracking::Initialize(){	
}

void RunTracking::Execute(){
	beaglebone->readData();
	SmartDashboard::PutBoolean("Beaglebone Connection", beaglebone->isConnected);
	SmartDashboard::PutNumber("Goal x", beaglebone->goalX);
	SmartDashboard::PutNumber("Goal Y", beaglebone->goalY);
	SmartDashboard::PutNumber("Goal angle", beaglebone->goalAngle);
	SmartDashboard::PutNumber("Ball x", beaglebone->ballX);
	SmartDashboard::PutNumber("Ball Y", beaglebone->ballY);
	SmartDashboard::PutNumber("Ball diam", beaglebone->ballDiam);
}

bool RunTracking::IsFinished(){
	return false;
}

void RunTracking::End(){
}

void RunTracking::Interrupted(){
	End();
}	
