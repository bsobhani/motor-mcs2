#ifndef SMARACT_MCS2_MOTOR_DRIVER_H
#define SMARACT_MCS2_MOTOR_DRIVER_H

/* Motor driver support for smarAct MCS RS-232 Controller   */

/* Derived from ACRMotorDriver.cpp by Mark Rivers, 2011/3/28 */

/* Author: Till Straumann <strauman@slac.stanford.edu>, 9/11 */

#ifdef __cplusplus

#include <asynMotorController.h>
#include <asynMotorAxis.h>
#include <stdarg.h>
#include <exception>
#include "SmarActControl.h"

enum SmarActMCS2ExceptionType {
	MCS2UnknownError,
	MCS2ConnectionError,
	MCS2CommunicationError,
};

class SmarActMCS2Exception : public std::exception {
public:
	SmarActMCS2Exception(SmarActMCS2ExceptionType t, const char *fmt, ...);
	SmarActMCS2Exception(SmarActMCS2ExceptionType t)
		: t_(t)
		{ str_[0] = 0; }
	SmarActMCS2Exception()
		: t_(MCS2UnknownError)
		{ str_[0] = 0; }
	SmarActMCS2Exception(SmarActMCS2ExceptionType t, const char *fmt, va_list ap);
	SmarActMCS2ExceptionType getType()
		const { return t_; }
	virtual const char *what()
		const throw() {return str_ ;}
protected:
	char str_[100];	
	SmarActMCS2ExceptionType t_;
};

class SmarActMCS2Axis : public asynMotorAxis
{
public:
	SmarActMCS2Axis(class SmarActMCS2Controller *cnt_p, int axis, int channel);
	asynStatus move(double position, int relative, double min_vel, double max_vel, double accel);
	asynStatus home(double min_vel, double max_vel, double accel, int forwards);
	asynStatus stop(double acceleration);
	asynStatus poll(bool* moving_p);
	friend class SmarActMCSController;
	int channel_;
private:
	SmarActMCS2Controller* c_p_;
	asynStatus comStatus_;

};

class SmarActMCS2Controller : public asynMotorController
{
public:
	SmarActMCS2Controller(const char *portName, const char *ipAddress, int ipPort, int numAxes, double movingPollPeriod, double idlePollPeriod);
	//virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
	//int64_t motorEncoderPosition_;
	//int64_t motorPosition_;

friend class SmarActMCS2Axis;
};

int main1();

#endif // _cplusplus
#endif // SMARACT_MCS2_MOTOR_DRIVER_H
