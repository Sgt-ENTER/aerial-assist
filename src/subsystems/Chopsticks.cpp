#include "Chopsticks.h"
#include "../Robotmap.h"

Chopsticks::Chopsticks():Subsystem("Chopsticks"){
    chopsticksmotor = new Victor (CHOPSTICK_MOTOR_PWM);
	verticalmotor = new Victor (VERTICAL_CHOPSTICKS_MOTOR_PWM);
}

Chopsticks::~Chopsticks() {
	delete chopsticksmotor;
	delete verticalmotor;
}

void Chopsticks::spinChopsticks(double speed) {
	chopsticksmotor->Set(speed); 
	
}

void Chopsticks::moveChopsticks(double speed){
	verticalmotor->Set(speed);
}
