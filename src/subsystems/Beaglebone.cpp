#include "Beaglebone.h"
#include "../Robotmap.h"
#include "../commands/RunTracking.h"

#include <cstdlib>

Beaglebone::Beaglebone():Subsystem ("Beaglebone"){
    connector = new TCPConnector();
    ballX = 99.0;
    ballY = 99.0;
    ballDiam = 99.0;
    goalX = 99.0;
    goalY = 99.0;
    goalAngle = 99.0;
    isConnected = false;
    isTrackingBall = true;
}

Beaglebone::~Beaglebone() {
	delete connector;
}

void Beaglebone::readData() {
	if (isTrackingBall){
		findBall();
	}
	else{
		findGoal();
	}
}

void Beaglebone::connect() {
	//this section opens a TCP connection to the beagle bone
	stream = connector->connect(BALL_VISION_IP,BALL_VISION_PORT);	
}

void Beaglebone::findGoal() {
	connect();
	if(stream){
		isConnected = true;
		
		char line[256];
		char* lastChar = NULL;
		
		
		stream->send("g", 1);
		
		stream->receive(line, sizeof(line));
		//this code gets the data from the beagle bone and converts into the following 
		// values so that the robot tracks and follows the ball
		double x = strtod(line, &lastChar);
		double y = strtod(lastChar, &lastChar);
		double angle = strtod(lastChar, NULL);
		goalX = x;
		goalY = y;
		goalAngle = angle;
		
		delete stream;
	}
	else{
		 isConnected = false;
	 }
}


void Beaglebone::findBall() {
	connect();
	if(stream){
		isConnected = true;
		
		char line[256];
		char* lastChar = NULL;
		
		DriverStation::Alliance colour;
	    colour = DriverStation::GetInstance()->GetAlliance();
	    if (colour == DriverStation::Alliance::kBlue){
		    stream->send("b", 1);
		}
		else{
			stream->send("r", 1);
		}
		stream->receive(line, sizeof(line));
		//this code gets the data from the beagle bone and converts into the following 
		// values so that the robot tracks and follows the ball
		double x = strtod(line, &lastChar);
		double y = strtod(lastChar, &lastChar);
		double diam = strtod(lastChar, NULL);
		ballX = x;
		ballY = y;
		ballDiam = diam;
		
		delete stream;
	}
	else{
		 isConnected = false;
	 }
}

void Beaglebone::InitDefaultCommand() {
    SetDefaultCommand(new RunTracking());
}

