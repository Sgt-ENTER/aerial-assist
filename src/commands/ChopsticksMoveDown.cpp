#include "ChopsticksMoveDown.h"
#include "../Robotmap.h"

ChopsticksMoveDown::ChopsticksMoveDown():CommandBase("ChopsticksMoveDown"){
	Requires(chopsticks);
}

void ChopsticksMoveDown::Initialize(){	
}

void ChopsticksMoveDown::Execute(){
	chopsticks->moveChopsticks(CHOPSTICKS_SPEED);
}

bool ChopsticksMoveDown::IsFinished(){
	return false;
}

void ChopsticksMoveDown::End(){
}

void ChopsticksMoveDown::Interrupted(){
	chopsticks->moveChopsticks (0.0);
}	

