#include "ChopsticksMoveUp.h"
#include "../Robotmap.h"

ChopsticksMoveUp::ChopsticksMoveUp():CommandBase("ChopsticksMoveUp"){
	Requires(chopsticks);
}

void ChopsticksMoveUp::Initialize(){	
}

void ChopsticksMoveUp::Execute(){
	chopsticks->moveChopsticks(CHOPSTICKS_SPEED);
}

bool ChopsticksMoveUp::IsFinished(){
	return false;
}

void ChopsticksMoveUp::End(){
}

void ChopsticksMoveUp::Interrupted(){
	chopsticks->moveChopsticks (0.0);
}	
