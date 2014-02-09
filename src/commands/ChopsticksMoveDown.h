#ifndef ChopsticksMoveDown_h
#define ChopsticksMoveDown_h

#include "../CommandBase.h"

class ChopsticksMoveDown: public CommandBase{
	public:
		ChopsticksMoveDown();
		virtual void Initialize();
		virtual void Execute();
		virtual bool IsFinished();
		virtual void End();
		virtual void Interrupted();
};

#endif
