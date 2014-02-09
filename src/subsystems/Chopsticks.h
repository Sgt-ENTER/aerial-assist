#ifndef Chopsticks_h
#define Chopsticks_h

#include <WPILib.h>

class Chopsticks: public Subsystem {
	public:
		Chopsticks();
		~Chopsticks();
		void spinChopsticks(double speed);
		void moveChopsticks(double speed);
		
	private:
		Victor* chopsticksmotor;
		Victor* verticalmotor;
};

#endif
