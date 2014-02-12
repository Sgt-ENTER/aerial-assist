#ifndef RunTracking_h
#define RunTracking_h

#include "../CommandBase.h"

class RunTracking: public CommandBase{
	public:
		RunTracking();
		virtual void Initialize();
		virtual void Execute();
		virtual bool IsFinished();
		virtual void End();
		virtual void Interrupted();
};

#endif
