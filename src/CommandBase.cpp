#include "CommandBase.h"
#include "Commands/Scheduler.h"

CommandBase::CommandBase(const char *name) : Command(name) {
}

CommandBase::CommandBase() : Command() {
}

// Initialize a single static instance of all of your subsystems to NULL
Chassis* CommandBase::chassis = NULL;
Chopsticks* CommandBase::chopsticks = NULL;
Catapult* CommandBase::catapult = NULL;
OI* CommandBase::oi = NULL;
Beaglebone* CommandBase::beaglebone = NULL;

void CommandBase::init() {
    // Create a single static instance of all of your subsystems. The following
	// line should be repeated for each subsystem in the project.
	chassis = new Chassis();
	chopsticks = new Chopsticks();
	catapult = new Catapult();
	beaglebone = new Beaglebone();
	
	// Always initialise the OI after all of the subsystems
	oi = new OI();
	// Nothing below this line!!
}
