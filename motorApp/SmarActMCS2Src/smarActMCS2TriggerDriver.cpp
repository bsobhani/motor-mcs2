#include "stdio.h"
#include "stdlib.h"
#include "smarActMCS2TriggerDriver.h"
#include <epicsString.h>
#include <epicsExport.h>
#include <epicsTypes.h>
#include "iocsh.h"
#include "asynPortDriver.h"


SmarActMCS2Trigger::SmarActMCS2Trigger(const char *portName, int maxArraySize, int channel)
	: asynPortDriver(portName,
				10,
				11,
				asynInt32Mask | asynFloat64Mask | asynEnumMask | asynDrvUserMask,
				asynInt32Mask | asynFloat64Mask | asynEnumMask,
				0,
				1,
				0,
				0)
{
	channel_=channel;
	//asynPrint(this->pasynUserSelf,ASYN_TRACE_FLOW,"New Trigger %s",portName);
	printf("New Trigger %s",portName);
	createParam(P_TriggerModeString,asynParamInt32,&P_TriggerMode);
	createParam(P_TriggerAPString,asynParamInt32,&P_TriggerAP);
	createParam(P_TriggerPWString,asynParamFloat64,&P_TriggerPW);
	createParam(P_TriggerIncString,asynParamFloat64,&P_TriggerInc);
	createParam(P_TriggerThresString,asynParamFloat64,&P_TriggerThres);
	createParam(P_TriggerDirString,asynParamInt32,&P_TriggerDir);



}

int set_trig_mode(int channel, int mode);
int set_trig_active_polarity(int channel, int ap);
int set_trig_pulse_width(int channel, int pw);

int set_trig_pc_inc(int channel, int64_t inc);
int set_trig_pc_thres(int channel, int64_t thres);
int set_trig_pc_dir(int channel, int dir);

asynStatus SmarActMCS2Trigger::writeInt32(asynUser* pasynUser, epicsInt32 value){
	const char* paramName;
	int result;
	getParamName(pasynUser->reason,&paramName);
	asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"writeInt32: %s %d\n",paramName,pasynUser->reason);
	if(pasynUser->reason==P_TriggerMode){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger mode setting to %d\n",value);
		result=set_trig_mode(channel_,value);
		printf("resultMode: %d",result);
	}
	if(pasynUser->reason==P_TriggerAP){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger active polarity setting to %d\n",value);
		result=set_trig_active_polarity(channel_,value);
		printf("resultAP: %d\n",result);
	}
	/*	
	if(pasynUser->reason==P_TriggerPW){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger pulse width setting to %d\n",value);
		set_trig_pulse_width(channel_,value);
	}
	*/
	if(pasynUser->reason==P_TriggerDir){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger direction setting to %d\n",value);
		result=set_trig_pc_dir(channel_,value);
		printf("resultTrigDir: %d\n",result);
	}
	/*
	if(pasynUser->reason==P_TriggerInc){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger increment setting to %d\n",value*1000);
		set_trig_pc_inc(channel_,value);
	}
	if(pasynUser->reason==P_TriggerThres){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger threshold setting to %d\n",value*1000);
		set_trig_pc_thres(channel_,value);
	}
	*/
	
	

	callParamCallbacks();
	return asynSuccess;
}
asynStatus SmarActMCS2Trigger::writeFloat64(asynUser* pasynUser, epicsFloat64 value){
	const char* paramName;
	const int time_scale_factor=1000000;
	const int pos_scale_factor=1000000;

	getParamName(pasynUser->reason,&paramName);
	asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"writeFloat64: %s %d\n",paramName,pasynUser->reason);
	
	if(pasynUser->reason==P_TriggerInc){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger increment setting to %d\n",(int64_t)value*1000);
		set_trig_pc_inc(channel_,(int64_t)(value*1000*pos_scale_factor));
	}
	if(pasynUser->reason==P_TriggerThres){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger threshold setting to %d\n",(int64_t)value*1000);
		set_trig_pc_thres(channel_,(int64_t)(value*1000*pos_scale_factor));
	}
	
	if(pasynUser->reason==P_TriggerPW){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger pulse width setting to %d\n",(int32_t)value);
		set_trig_pulse_width(channel_,(int32_t)(value*time_scale_factor));
		printf("HERE\n");
	}

	callParamCallbacks();
	return asynSuccess;
}

/*
asynStatus SmarActMCS2Trigger::writeInt64(asynUser* pasynUser, epicsInt64 value){
	const char* paramName;
	getParamName(pasynUser->reason,&paramName);
	asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"%s %d\n",paramName,pasynUser->reason);
	if(pasynUser->reason==P_TriggerInc){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger increment setting to %d\n",value);
		set_trig_pc_inc(channel_,value);
	}
	if(pasynUser->reason==P_TriggerThres){
		asynPrint(pasynUserSelf,ASYN_TRACE_FLOW,"Trigger threshold setting to %d\n",value);
		set_trig_pc_thres(channel_,value);
	}
	
	callParamCallbacks();
	return asynSuccess;
}
*/
void SmarActMCS2CreateTrigger(const char* portName,int channel){
	new SmarActMCS2Trigger(portName, 0, channel);
}

void cc_ft(const iocshArgBuf* args){
	SmarActMCS2CreateTrigger(args[0].sval,args[1].ival);
}

static const iocshArg ta_a0 = {"portName", iocshArgString};
static const iocshArg ta_a1 = {"channel", iocshArgInt};
static const iocshArg * const ta_as[] = {&ta_a0, &ta_a1};

iocshFuncDef ta_def = {"smarActMCS2CreateTrigger",2,ta_as};

