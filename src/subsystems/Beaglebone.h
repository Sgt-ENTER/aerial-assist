#ifndef Beaglebone_h
#define Beaglebone_h

#include "../net/tcpconnector.h"

#include <WPILib.h>

class Beaglebone: public Subsystem{
	public:
		Beaglebone();
		~Beaglebone();
		void readData();
		double ballX, ballY, ballDiam, goalX, goalY, goalAngle;
		bool isConnected;
		bool isTrackingBall;
		void InitDefaultCommand();
		void shutdown();
				
	private:
		TCPConnector* connector;
		TCPStream* stream;
		void connect();
		void findGoal();
		void findBall();
	
};


#endif
