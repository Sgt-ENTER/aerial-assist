#ifndef ChopsticksMoveUp_h
#define ChopsticksMoveUp_h

#include "../CommandBase.h"

class ChopsticksMoveUp: public CommandBase{
	public:
		ChopsticksMoveUp();
		virtual void Initialize();
		virtual void Execute();
		virtual bool IsFinished();
		virtual void End();
		virtual void Interrupted();
};

#endif

