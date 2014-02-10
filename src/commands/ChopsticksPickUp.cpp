
#include "ChopsticksPickUp.h"
#include "../Robotmap.h"

ChopsticksPickUp::ChopsticksPickUp():CommandBase("ChopsticksPickUp"){
	Requires(chopsticks);
}

void ChopsticksPickUp::Initialize(){	
}

void ChopsticksPickUp::Execute(){
	chopsticks->spinChopsticks(CHOPSTICKS_ROTATE_SPEED);
}

bool ChopsticksPickUp::IsFinished(){
	return false;
}

void ChopsticksPickUp::End(){
}

void ChopsticksPickUp::Interrupted(){
	chopsticks->spinChopsticks (0.0);
}	
