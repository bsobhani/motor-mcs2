/**
 * Motor record driver for SmarAct MCS2 controller
 *
 * Author: Alex Sobhani
 *
 * Adapted from SmarAct MCS motor record driver by Till Straumann
 *
 */


#include <iocsh.h>

#include <asynOctetSyncIO.h>
#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <smarActMCS2MotorDriver.h>
#include <errlog.h>
#include "SmarActControl.h"
#include "smarActMCS2TriggerDriver.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <exception>

#include <math.h>

#include <epicsString.h>
#include <epicsExport.h>

#include "iocsh.h"
#include "asynPortDriver.h"
#include <epicsTypes.h>

enum SmarActMCS2Status {
	Stopped     = 0,
	Stepping    = 1,
	Scanning    = 2,
	Holding     = 3,
	Targeting   = 4,
	MoveDelay   = 5,
	Calibrating = 6,
	FindRefMark = 7,
	Locked      = 9
};

SmarActMCS2Exception::SmarActMCS2Exception(SmarActMCS2ExceptionType t, const char *fmt, ...)
	: t_(t)
{
va_list ap;
	if ( fmt ) {
		va_start(ap, fmt);
		epicsVsnprintf(str_, sizeof(str_), fmt, ap);
		va_end(ap);
	} else {
		str_[0] = 0;
	}
};

SmarActMCS2Exception::SmarActMCS2Exception(SmarActMCS2ExceptionType t, const char *fmt, va_list ap)
		: t_(t)
{
	epicsVsnprintf(str_, sizeof(str_), fmt, ap);
}

int initmcs(const char *IPAddress, int IPPort);

/** Creates a new SmarActMCS2Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] IPAddress         The controller's ip address
  * \param[in] IPPort            TCP/IP port used to communicate with the controller 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
SmarActMCS2Controller::SmarActMCS2Controller(const char *portName, const char *IPAddress, int IPPort, int numAxes, double movingPollPeriod, double idlePollPeriod)
	: asynMotorController(portName, numAxes,
	                      0, // parameters
	                      0, // interface mask
	                      0, // interrupt mask
	                      ASYN_CANBLOCK | ASYN_MULTIDEVICE,
	                      1, // autoconnect
	                      0,0) // default priority and stack size
	
{

	if(initmcs(IPAddress, IPPort)!=0){	
	}
	startPoller(movingPollPeriod,idlePollPeriod,0);

}

asynStatus getp(int channel, int64_t* position);
bool is_moving(int channel);

void trig1_to_5();
void stop(int8_t channel);
asynStatus getstatus(int channel, int32_t* state);
asynStatus SmarActMCS2Axis::stop(double acceleration){
	::stop(channel_);
	return asynSuccess;
}

asynStatus SmarActMCS2Axis::poll(bool* moving_p){
	int64_t val;
	comStatus_=getp(channel_,&val);
	if(comStatus_!=asynSuccess){
		*moving_p=false;
		asynPrint(c_p_->pasynUserSelf,ASYN_TRACE_ERROR,"Error reading axis, channel=%d, comstatus=%d!\n", channel_,comStatus_);
		setIntegerParam(c_p_->motorStatusProblem_,1);
		setIntegerParam(c_p_->motorStatusCommsError_,1);
		setIntegerParam(c_p_->motorStatusMoving_,*moving_p);
		setIntegerParam(c_p_->motorStatusDone_, ! *moving_p);
		//initmcs();
		callParamCallbacks();
		return comStatus_;
	}
	//printf("SUCCESS\n\n");
	int32_t state;
	getstatus(channel_, &state);
	//printf("State: %d\n",state);
	//printf("Following: %d",state & SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED);
	setIntegerParam(c_p_->motorStatusHighLimit_, (state & SA_CTL_CH_STATE_BIT_END_STOP_REACHED)/SA_CTL_CH_STATE_BIT_END_STOP_REACHED);
	setIntegerParam(c_p_->motorStatusLowLimit_, (state & SA_CTL_CH_STATE_BIT_END_STOP_REACHED)/SA_CTL_CH_STATE_BIT_END_STOP_REACHED);
	setIntegerParam(c_p_->motorStatusFollowingError_, (state & SA_CTL_CH_STATE_BIT_FOLLOWING_LIMIT_REACHED)/SA_CTL_CH_STATE_BIT_FOLLOWING_LIMIT_REACHED);
	/*if(state & SA_CTL_CH_STATE_BIT_END_STOP_REACHED){
		printf("Limit switch reached\n");
		stop(0);
	}*/
	double accel;
	val=val/1000;
	setDoubleParam(c_p_->motorEncoderPosition_,(double)val);
	setDoubleParam(c_p_->motorPosition_,(double)val);
	if(is_moving(channel_)==true){
		*moving_p=true;
	}
	else{
		*moving_p=false;
	}
	setIntegerParam(c_p_->motorStatusProblem_,0);
	setIntegerParam(c_p_->motorStatusCommsError_,0);

	setIntegerParam(c_p_->motorStatusMoving_,*moving_p);
	setIntegerParam(c_p_->motorStatusDone_, ! *moving_p);
	callParamCallbacks();
	return asynSuccess;
}
SmarActMCS2Axis::SmarActMCS2Axis(class SmarActMCS2Controller *cnt_p, int axis, int channel)
	: asynMotorAxis(cnt_p, axis), c_p_(cnt_p)
{
	channel_=channel;
}

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
static const iocshArg cc_a0 = {"Port name [string]",               iocshArgString};
static const iocshArg cc_a1 = {"IP Address [string]",               iocshArgString};
static const iocshArg cc_a2 = {"IP port [int]",               iocshArgInt};
//static const iocshArg cc_a1 = {"I/O port name [string]",           iocshArgString};
static const iocshArg cc_a3 = {"Number of axes [int]",             iocshArgInt};
static const iocshArg cc_a4 = {"Moving poll period (s) [double]",  iocshArgDouble};
static const iocshArg cc_a5 = {"Idle poll period (s) [double]",    iocshArgDouble};

static const iocshArg * const cc_as[] = {&cc_a0, &cc_a1, &cc_a2, &cc_a3, &cc_a4, &cc_a5};

static const iocshFuncDef cc_def = {"smarActMCS2CreateController", sizeof(cc_as)/sizeof(cc_as[0]), cc_as};



asynStatus do_a_move(int64_t distance, int64_t velocity, int channel);
void set_accel(int64_t accel, int channel);
void findReference(int8_t channel);

asynStatus  
SmarActMCS2Axis::move(double position, int relative, double min_vel, double max_vel, double accel)
{
	if(comStatus_!=asynSuccess){
		printf("Move Error\n");
		return asynError;
	}
	asynPrint(c_p_->pasynUserSelf,ASYN_TRACE_FLOW,"Moving to %f, velo=%f, accel=%f, channel=%d!\n", position,max_vel,accel,channel_);
	set_accel((int64_t)accel*1000,channel_);
	do_a_move((int64_t) position*1000, (int64_t) max_vel*1000, channel_);
	return asynSuccess;
} 

asynStatus  
SmarActMCS2Axis::home(double min_vel, double max_vel, double accel, int forwards)
{
	findReference(channel_);
	return asynSuccess;
} 


extern "C" void *
smarActMCS2CreateController(
	const char *motorPortName,
	//const char *ioPortName,
	const char *ipAddress,
	int ipPort,
	int         numAxes,
	double      movingPollPeriod,
	double      idlePollPeriod)
{
	
	return new SmarActMCS2Controller(motorPortName,ipAddress,ipPort,numAxes,movingPollPeriod,idlePollPeriod);
	//return 0;
}

static void cc_fn(const iocshArgBuf *args)
{
	smarActMCS2CreateController(
		args[0].sval,
		args[1].sval,
		args[2].ival,
		args[3].ival,
		args[4].dval,
		args[5].dval);
}


static const iocshArg ca_a0 = {"Controller Port name [string]",    iocshArgString};
static const iocshArg ca_a1 = {"Axis number [int]",                iocshArgInt};
static const iocshArg ca_a2 = {"Channel [int]",                    iocshArgInt};

static const iocshArg * const ca_as[] = {&ca_a0, &ca_a1, &ca_a2};

/* iocsh wrapping and registration business (stolen from ACRMotorDriver.cpp) */
/* smarActMCS2CreateAxis called to create each axis of the smarActMCS2 controller*/
static const iocshFuncDef ca_def = {"smarActMCS2CreateAxis", 3, ca_as};

extern "C" void *
smarActMCS2CreateAxis(
	const char *controllerPortName,
	int        axisNumber,
	int        channel)
{
	SmarActMCS2Controller *pC;
	pC = (SmarActMCS2Controller*) findAsynPortDriver(controllerPortName);
	if (!pC) {
		printf("smarActMCS2CreateAxis: Error port %s not found\n", controllerPortName);
		return 0;
	}
	pC->lock();
	SmarActMCS2Axis* sa = new SmarActMCS2Axis(pC, axisNumber, channel);
	pC->unlock();
	//sa->move(1,2,3,4,5);
	return sa;
}

static void ca_fn(const iocshArgBuf *args)
{
	smarActMCS2CreateAxis(
		args[0].sval,
		args[1].ival,
		args[2].ival);
}
void cc_ft(const iocshArgBuf* args);
extern iocshFuncDef ta_def;

static void smarActMCS2MotorRegister(void)
{
  iocshRegister(&cc_def, cc_fn);  // smarActMCS2CreateController
  iocshRegister(&ca_def, ca_fn);  // smarActMCS2CreateAxis
  iocshRegister(&ta_def, cc_ft);  // smarActMCS2CreateTrigger 
}

extern "C" {
epicsExportRegistrar(smarActMCS2MotorRegister);
}
