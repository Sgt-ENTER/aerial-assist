
#include "RunTracking.h"
#include "../Robotmap.h"

RunTracking::RunTracking():CommandBase("RunTracking"){
	Requires(beaglebone);
}

void RunTracking::Initialize(){	
}

void RunTracking::Execute(){
	beaglebone->readData();
}

bool RunTracking::IsFinished(){
	return false;
}

void RunTracking::End(){
}

void RunTracking::Interrupted(){
	End();
}	
