#ifndef SMARACT_MCS2_TRIGGER_DRIVER_H
#define SMARACT_MCS2_TRIGGER_DRIVER_H

#include "asynPortDriver.h"
#include "epicsTypes.h"

#define P_TriggerModeString	"MCS2_TRIG_MODE"
#define P_TriggerAPString	"MCS2_TRIG_AP"
#define P_TriggerPWString	"MCS2_TRIG_PW"
#define P_TriggerIncString	"MCS2_TRIG_INC"
#define P_TriggerThresString	"MCS2_TRIG_THRES"
#define P_TriggerDirString	"MCS2_TRIG_DIR"



class SmarActMCS2Trigger : public asynPortDriver{
public:
	SmarActMCS2Trigger(const char *portName, int maxArraySize, int channel);
	virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
	//virtual asynStatus writeInt64(asynUser* pasynUser, epicsInt64 value);
	virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);
	int P_TriggerMode;
	int P_TriggerAP;
	int P_TriggerPW;
	int P_TriggerInc;
	int P_TriggerThres;
	int P_TriggerDir;
	int channel_;
};

#endif
