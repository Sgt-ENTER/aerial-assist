#include "ChopsticksPutDown.h"
#include "../Robotmap.h"

ChopsticksPutDown::ChopsticksPutDown():CommandBase("ChopsticksPutDown"){
	Requires(chopsticks);
}

void ChopsticksPutDown::Initialize(){	
}

void ChopsticksPutDown::Execute(){
	chopsticks->spinChopsticks(-CHOPSTICKS_ROTATE_SPEED);
}

bool ChopsticksPutDown::IsFinished(){
	return false;
}

void ChopsticksPutDown::End(){
}

void ChopsticksPutDown::Interrupted(){
	chopsticks->spinChopsticks (0.0);
}	
