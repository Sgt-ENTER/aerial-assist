#include "CatchBall.h"
#include "../subsystems/Chassis.h"
#include "../Robotmap.h"




CatchBall::CatchBall(): CommandBase("CatchBall"){
	Requires(chassis);
}

void CatchBall::Initialize(){
}

void CatchBall::Execute(){
}

bool CatchBall::IsFinished(){
	return false; 
}

void CatchBall::End(){
	
}

void CatchBall::Interrupted(){
	
}


