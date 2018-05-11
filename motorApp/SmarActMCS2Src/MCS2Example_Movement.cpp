/*
* SmarAct MCS2 programming example: Movement
*
* This programming example shows you how to 
* find available MCS2 devices to connect to
* and how to perform different movement commands.
*
* For a full command reference see the MCS2 Programmers Guide.
*
*/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "SmarActControl.h"
#include "smarActMCS2MotorDriver.h"

#if defined(_WIN32)
#define strtok_r strtok_s
#endif

void calibrate(int8_t channel);
void findReference(int8_t channel);
void move(int8_t channel, int32_t moveMode, int8_t direction);
void stop(int8_t channel);

SA_CTL_DeviceHandle_t dHandle;

void move_r(int8_t channel, int32_t moveMode, int8_t direction, int64_t distance);
 
void exitOnError(SA_CTL_Result_t result) {     
    if (result != SA_CTL_ERROR_NONE) {
        SA_CTL_Close(dHandle);
        // Passing an error code to "SA_CTL_GetResultInfo" returns a human readable string
        // specifying the error.
        printf("MCS2 %s (error: 0x%04x).\nPress return to exit.\n", SA_CTL_GetResultInfo(result), result);
        getchar();
        exit(1);
    }
}

void printMenu(void) {
    printf("\n*******************************************************\n");
    printf("WARNING: make sure the positioner can move freely\n \
            without damaging other equipment!\n");
    printf("*******************************************************\n");
    printf("\nEnter command and return:\n");
    printf("[?] print this menu\n");
    printf("[c] calibrate\n");
    printf("[f] find reference\n");
    printf("[+] perform movement in positive direction\n");
    printf("[-] perform movement in negative direction\n");
    printf("[s] stop\n");
    printf("[p] get current position\n");
    printf("[0] set move mode: closed loop absolute move\n");
    printf("[1] set move mode: closed loop relative move\n");
    printf("[2] set move mode: open loop scan absolute\n");
    printf("[3] set move mode: open loop scan relative\n");
    printf("[4] set move mode: open loop step\n");
    printf("[5] set control mode: standard mode\n");
    printf("[6] set control mode: quiet mode\n");
    printf("[q] quit\n");
}

void trig1_to_5(){
	SA_CTL_Result_t result1,result2;
	result1 = SA_CTL_SetProperty_i32(dHandle,0,SA_CTL_PKEY_IO_MODULE_VOLTAGE, SA_CTL_IO_MODULE_VOLTAGE_5V);

	result2 = SA_CTL_SetProperty_i32(dHandle,0,SA_CTL_PKEY_IO_MODULE_OPTIONS, SA_CTL_IO_MODULE_OPT_BIT_ENABLED);
	
result1 = SA_CTL_SetProperty_i32(dHandle,0,SA_CTL_PKEY_CH_OUTPUT_TRIG_POLARITY, SA_CTL_TRIGGER_POLARITY_ACTIVE_LOW);


	result1 = SA_CTL_SetProperty_i32(dHandle,0,SA_CTL_PKEY_CH_OUTPUT_TRIG_MODE, SA_CTL_CH_OUTPUT_TRIG_MODE_CONSTANT);

//printf("%d %d\n",result1,result2);
}

int set_trig_mode(int channel, int mode){
	// mode = 0 : constant
	// mode = 1 : position compare
	// mode = 2 : target reached
	// mode = 3 : actively moving

	int result = SA_CTL_SetProperty_i32(dHandle,channel,SA_CTL_PKEY_CH_OUTPUT_TRIG_MODE, mode);
	return result;

}

int set_trig_pulse_width(int channel, int pw){

	//printf("PW: %d\n",pw);
	int result = SA_CTL_SetProperty_i32(dHandle,channel,SA_CTL_PKEY_CH_OUTPUT_TRIG_PULSE_WIDTH, pw);
	return result;

}
int set_trig_pc_inc(int channel, int64_t inc){

	int result = SA_CTL_SetProperty_i64(dHandle,channel,SA_CTL_PKEY_CH_POS_COMP_INCREMENT, inc);
	return result;

}
int set_trig_pc_thres(int channel, int64_t thres){

	int result = SA_CTL_SetProperty_i64(dHandle,channel,SA_CTL_PKEY_CH_POS_COMP_START_THRESHOLD, thres);
	return result;

}
int set_trig_pc_dir(int channel, int dir){
	int result;
	if(dir==0){
		result = SA_CTL_SetProperty_i32(dHandle,channel,SA_CTL_PKEY_CH_POS_COMP_DIRECTION, SA_CTL_FORWARD_DIRECTION);
	}
	else if(dir==1){
		result = SA_CTL_SetProperty_i32(dHandle,channel,SA_CTL_PKEY_CH_POS_COMP_DIRECTION,SA_CTL_BACKWARD_DIRECTION);
	}
	else if(dir==2){
		result = SA_CTL_SetProperty_i32(dHandle,(int8_t)channel,SA_CTL_PKEY_CH_POS_COMP_DIRECTION,SA_CTL_EITHER_DIRECTION);
		printf("EITHER DIRECTION!!!!!\n");
	}
	return result;

}
int set_trig_active_polarity(int channel, int ap){
	// ap = 0 : active polarity low
	// ap = 1 : active polarity high

	SA_CTL_Result_t result1,result2;
	int module;
	module=channel/3;
	result1 = SA_CTL_SetProperty_i32(dHandle,module,SA_CTL_PKEY_IO_MODULE_VOLTAGE, SA_CTL_IO_MODULE_VOLTAGE_5V);

	result2 = SA_CTL_SetProperty_i32(dHandle,module,SA_CTL_PKEY_IO_MODULE_OPTIONS, SA_CTL_IO_MODULE_OPT_BIT_ENABLED);
	if(ap==0){
		result1 = SA_CTL_SetProperty_i32(dHandle,channel,SA_CTL_PKEY_CH_OUTPUT_TRIG_POLARITY, SA_CTL_TRIGGER_POLARITY_ACTIVE_LOW);
	}

	else{
		result1 = SA_CTL_SetProperty_i32(dHandle,channel,SA_CTL_PKEY_CH_OUTPUT_TRIG_POLARITY, SA_CTL_TRIGGER_POLARITY_ACTIVE_HIGH);
	}	
	//result1 = SA_CTL_SetProperty_i32(dHandle,0,SA_CTL_PKEY_CH_OUTPUT_TRIG_MODE, SA_CTL_CH_OUTPUT_TRIG_MODE_CONSTANT);
	return result1;
//printf("%d %d\n",result1,result2);
}

int initialized=0;

int initmcs(const char* IPAddress, int IPPort){
    if(initialized==1){
        SA_CTL_Close(dHandle);
    }
    SA_CTL_Result_t result;

    const char *version = SA_CTL_GetFullVersionString();
    //char *locator = "network:192.168.1.200:55550";
    //char *locator = "network:192.168.1.200:55550";
    char locator[500];
    sprintf(locator,"network:%s:%d",IPAddress,IPPort);
    
    result = SA_CTL_Open(&dHandle, locator, "");
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to open \"%s\".\n",locator);
	return 1;
    }
    
    int8_t channel = 0;
    
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MAX_CL_FREQUENCY, 6000);
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_HOLD_TIME, 1000);


    int32_t moveMode = SA_CTL_MOVE_MODE_CL_ABSOLUTE;
    int64_t position;
    char key;
    moveMode = SA_CTL_MOVE_MODE_CL_RELATIVE;
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 1000000000);
    initialized=1;
    return result;
}

void set_accel(int64_t accel, int channel){
	SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, accel);
}

asynStatus do_a_move(int64_t distance, int64_t velocity, int channel){
    int result;

    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, velocity);
    int32_t moveMode = SA_CTL_MOVE_MODE_CL_ABSOLUTE;
    move_r(channel, moveMode, 0, distance);
    return asynSuccess;
}

asynStatus getp(int channel, int64_t* val){
    int result = SA_CTL_GetProperty_i64(dHandle, channel, SA_CTL_PKEY_POSITION, val,0);
    if(result==0)
    	return asynSuccess;
    return asynError;

}

asynStatus getstatus(int channel, int32_t* state){
    int result;
    result = SA_CTL_GetProperty_i32(dHandle, channel, SA_CTL_PKEY_CHANNEL_STATE, state,0);
    if(result==0)
    	return asynSuccess;
    return asynError;

}



bool is_moving(int channel){
	bool moving;
	int32_t state;
	SA_CTL_GetProperty_i32(dHandle,channel,SA_CTL_PKEY_CHANNEL_STATE,&state,0);
	if (state & SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING){
		return true;
	}
	return false;
}

int main1() {

    SA_CTL_Result_t result;

    printf("*******************************************************\n");
    printf("*  SmarAct MCS2 Programming Example (Movement)        *\n");
    printf("*******************************************************\n");

    // Read the version of the DLL
    // Note: this is the only function that does not require the DLL to be initialized.
    const char *version = SA_CTL_GetFullVersionString();
    printf("SmarActCTL DLL version: %s.\n", version);

    // MCS2 devices are identified with locator strings.
    // To list all available devices we use the "SA_CTL_FindDevices" function.
    // The returned device list contains the locators of all found MCS2 devices separated by newlines.
    // The "ioDeviceListLen" parameter must contain the size of the passed deviceList buffer and
    // contains the length of the deviceList when the function returns.
    // In case the deviceList size exceed the provided buffer size an "SA_CTL_QUERYBUFFER_SIZE_ERROR" is returned.
    /*char deviceList[1024];
    size_t ioDeviceListLen = sizeof(deviceList);
    result = SA_CTL_FindDevices("", deviceList, &ioDeviceListLen);
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to find devices.\n");
    }
    exitOnError(result);
    if (strlen(deviceList) == 0) {
        printf("MCS2 no devices found. Exit.\n");
        getchar();
        exit(1);
    } else {
        printf("MCS2 available devices:\n%s\n", deviceList);
    }
    // Open the first MCS2 device from the list
    char *ptr;
    strtok_r(deviceList, "\n",&ptr);*/
    //char *locator = deviceList; 
    char *locator = "network:192.168.1.200:55550";
    result = SA_CTL_Open(&dHandle, locator, "");
    if (result != SA_CTL_ERROR_NONE) {
        printf("MCS2 failed to open \"%s\".\n",locator);
    }
    exitOnError(result);
    printf("MCS2 opened \"%s\".\n", locator);

    // There are three commands which trigger a movement of the positioner:
    // "SA_CTL_Calibrate", "SA_CTL_Reference" and "SA_CTL_Move".
    // The command "SA_CTL_Stop" aborts any ongoing movement.

    // Note that movement commands are always non-blocking. This means that the function call 
    // returns immediatelly without waiting for the actual movement to complete. In addition,
    // they do not return an error in case the movement can not be processed for some reason,
    // e.g. if there is no positioner connected for the given channel. 
    // The channel state can be polled to determine the moment when the movement finished.
    // Alternativly, channel events can be used to receive "commandFinished" events.
    // Both methodes are not show in this example. Instead, refer to the further programming examples.

    // Generally the appropriate movement properties (like mode, velocity, acceleration, etc.) 
    // should be set before starting the actual movement.

    // The following code shows the calibration sequence, the find reference function, closed loop
    // movement and open loop movement.
    // See the corresponding functions "calibrate","findReference" and "move" at the
    // end of this file for more information on the specific properties.
  
    // We assume that there is a linear positioner with integrated sensor connected to channel 0.
    int8_t channel = 0;
    
    // We start by setting some general channel properties.
    // Set max closed loop frequency (maxCLF) to 6 kHz. This properties sets a limit for the maximum actuator driving frequency.
    // The maxCLF is not persistent thus set to a default value at startup.
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MAX_CL_FREQUENCY, 6000);
    exitOnError(result);
    // The hold time specifies how long the position is actively held after reaching the target.
    // This property is also not persistent and set to zero by default.
    // A value of 0 deactivates the hold time feature, a value of SA_CTL_INFINITE (0xffffffff) sets the time to infinite.
    // (Until manually stopped by "SA_CTL_Stop") Here we set the hold time to 1000 ms.
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_HOLD_TIME, 1000);
    exitOnError(result);

    // The move mode states the type of movement performed when sending the "SA_CTL_Move" command.
    int32_t moveMode = SA_CTL_MOVE_MODE_CL_ABSOLUTE;
    int64_t position;
    char key;
    printMenu();
    do {
        key = getchar();
        switch (key) {
            case 'c':
                calibrate(channel);
                break;
            case 'f':
                findReference(channel);
                break;
            case '+':
                move(channel, moveMode, 0);
                break;
            case '-':
                move(channel, moveMode, 1);
                break;
            case 's':
                stop(channel);
                break;
            case 'p':
                result = SA_CTL_GetProperty_i64(dHandle, channel, SA_CTL_PKEY_POSITION, &position,0);
                exitOnError(result);
                printf("MCS2 position: %lld pm.\n", position);
                break;
            case '0':
                moveMode = SA_CTL_MOVE_MODE_CL_ABSOLUTE;
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, moveMode);
                exitOnError(result);
                printf("MCS2 set closed-loop absolute move mode\n");
                break;
            case '1':
                moveMode = SA_CTL_MOVE_MODE_CL_RELATIVE;
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, moveMode);
                exitOnError(result);
                printf("MCS2 set closed-loop relative move mode\n");
                break;
            case '2':
                moveMode = SA_CTL_MOVE_MODE_SCAN_ABSOLUTE;
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, moveMode);
                exitOnError(result);
                printf("MCS2 set open-loop scan absolute move mode\n");
                break;
            case '3':
                moveMode = SA_CTL_MOVE_MODE_SCAN_RELATIVE;
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, moveMode);
                exitOnError(result);
                printf("MCS2 set open-loop scan relative move mode\n");
                break;
            case '4':
                moveMode = SA_CTL_MOVE_MODE_STEP;
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, moveMode);
                exitOnError(result);
                printf("MCS2 set open-loop step move mode\n");
                break;
            case '5':
                // In the "normal" actuator mode the control loop generates actuator driving signals
                // with a maximum frequency limited by the max closed loop frequency (maxCLF).
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_NORMAL);
                exitOnError(result);
                printf("MCS2 set normal actuator mode\n");
                break;
            case '6':
                // The "quiet" actuator mode allows positioner movement with minimal noise emission.
                result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_ACTUATOR_MODE, SA_CTL_ACTUATOR_MODE_QUIET);
                exitOnError(result);
                printf("MCS2 set quiet actuator mode\n");
                break;
            case 'q':
                break;
            case '?':
                printMenu();
                break;
        }
    } while (key != 'q');

    // Before closing the program the connection to the device must be closed by calling "SA_CTL_Close".
    SA_CTL_Close(dHandle);
    printf("MCS2 close.\n");
    printf("*******************************************************\n");
    return 0;
}

// CALIBRATION
// The calibration sequence is used to increase the precision of the position calculation. This function
// should be called once for each channel if the mechanical setup changes.
// (e.g. a different positioner was connected to the channel, the positioner type was set to a different type) 
// The calibration data will be saved to non-volatile memory, thus it is not necessaray to perform the calibration sequence
// on each initialization.
// Note: the "CH_STATE_BIT_IS_CALIBRATED" in the channel state can be used to determine
// if valid calibration data is stored for the specific channel.

// During the calibration sequence the positioner performs a movement of up to several mm, make sure to not start
// the calibration near a mechanical endstop in order to ensure proper operation.
// See the MCS2 Programmers Guide for more information on the calibration.
void calibrate(int8_t channel) {
    SA_CTL_Result_t result;
    printf("MCS2 start calibration on channel: %d.\n", channel);
    // Set calibration options (start direction: forward)
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_CALIBRATION_OPTIONS, 0);
    exitOnError(result);
    // Start calibration sequence
    result = SA_CTL_Calibrate(dHandle, channel, 0);
    // Note that the function call returns immediately, without waiting for the movement to complete.
    // The "CH_STATE_BIT_CALIBRATING" in the channel state can be monitored to determine 
    // the end of the calibration sequence.
    exitOnError(result);
}

// FIND REFERENCE
// Since the position sensors work on an incremental base, the referencing sequence is used to 
// establish an absolute positioner reference for the positioner after system startup.
// Note: the "CH_STATE_BIT_PHYS_POSITION_KNOWN" in the channel state can be used to to decide 
// whether it is necessary to perform the referencing sequence.
void findReference(int8_t channel) {
    SA_CTL_Result_t result;
    printf("MCS2 find reference on channel: %d.\n", channel);
    // Set find reference options.
    // The reference options specify the behaviour of the find reference sequence.
    // The reference flags can be ORed to build the reference options.
    // By default (options = 0) the positioner returns to the position of the reference mark.
    // Note: In contrast to previous controller systems this is not mandatory.
    // The MCS2 controller is able to find the reference position "on-the-fly".
    // See the MCS2 Programmer Guide for a description of the different modes.
    result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_REFERENCING_OPTIONS, 0);
    exitOnError(result);
    // Set velocity to 1mm/s
    result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 1000000000);
    exitOnError(result);
    // Setting the acceleration to zero disables the acceleration control.

   result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 0);
    exitOnError(result);
    // Start referencing sequence
    result = SA_CTL_Reference(dHandle, channel, 0);
    // Note that the function call returns immediatelly, without waiting for the movement to complete.
    // The "CH_STATE_BIT_FINDING_REFERENCE" in the channel state can be monitored to determine 
    // the end of the referencing sequence.
    exitOnError(result);
}

// MOVE
// The move command instructs a positioner to perform a movement.
// The given "moveValue" parameter is interpreted according to the previously configured move mode.
// It can be a position value (in case of closed loop movement mode), a scan value (in case of scan move mode)
// or a number of steps (in case of step move mode).


void move_r(int8_t channel, int32_t moveMode, int8_t direction, int64_t distance) {
    SA_CTL_Result_t result;
    int64_t moveValue;
    // Set move mode depending properties for the next movement. 
    switch (moveMode) {
        case SA_CTL_MOVE_MODE_CL_ABSOLUTE:
	    SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_MOVE_MODE, moveMode);
            // Set move velocity [in pm/s].
            //result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 1000000000);
            exitOnError(result);
            // Set move acceleration [in pm/s2].
            //result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 1000000000);
            exitOnError(result);
            // Specify absolute position [in pm].
            if (direction) moveValue = -2000000000;
            else moveValue = 1000000000;
            moveValue=distance;
            //printf("move_r: MCS2 move channel %d to absolute position: %lld pm.\n", channel, moveValue);
            break;
        case SA_CTL_MOVE_MODE_CL_RELATIVE:
            // Set move velocity [in pm/s].
            result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 500000000);
            exitOnError(result);
            // Set move acceleration [in pm/s2], a value of 0 disables the acceleration control.
            result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 0);
            exitOnError(result);
            // Specify relative position distance [in pm] and direction.
            moveValue = 500000000;
            if (direction) moveValue = -moveValue;
            printf("MCS2 move channel %d relative: %lld pm.\n", channel, moveValue);
            break;
        case SA_CTL_MOVE_MODE_SCAN_ABSOLUTE:
            // Set scan velocity [in dac increments/s].
            // Valid range: 1 to 65535000000
            result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_SCAN_VELOCITY, (65535*2));
            exitOnError(result);
            // Specify absolute scan target to which to scan to [in dac increments].
            // Valid range: 0 to 65535 corresponding to 0 to 100V piezo voltage
            if (direction) moveValue = 0;
            else moveValue = 65535;
            printf("MCS2 scan channel %d absolute to: %lld.\n", channel, moveValue);
            break;
        case SA_CTL_MOVE_MODE_SCAN_RELATIVE:
            // Set scan velocity [in dac increments/s].
            result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_SCAN_VELOCITY, 65535);
            exitOnError(result);
            // Specify relative scan target and direction to which to scan to [in dac increments].
            // Valid range: -65535 to 65535 corresponding to 0 to 100V piezo voltage
            // If the resulting absolute scan target exceeds the valid range the scan movement will stop at the boundary.
            moveValue = 65535;
            if (direction) moveValue = -moveValue;
            printf("MCS2 scan channel %d relative: %lld.\n", channel, moveValue);
            break;
        case SA_CTL_MOVE_MODE_STEP:
            // Set step frequency [in Hz].
            // Valid range: 1 to 20000 Hz
            result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_STEP_FREQUENCY, 1000);
            exitOnError(result);
            // Set maximum step amplitude [in dac increments].
            // valid range: 0 to 65535 corresponding to 0 to 100V piezo voltage
            // Lower amplitude values result in smaller step width.
            result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_STEP_AMPLITUDE, 65535);
            exitOnError(result);
            // Specify the number of steps to perform and the direction.
            //moveValue = 500;
            if (direction) moveValue = -moveValue;
            printf("MCS2 open loop step move, channel %d, steps: %lld.\n", channel, moveValue);
            break;
    }
    // Start actual movement.
    result = SA_CTL_Move(dHandle, channel, distance, 0);
    // Note that the function call returns immediatelly, without waiting for the movement to complete.
    // The "CH_STATE_BIT_ACTIVELY_MOVING" and "CH_STATE_BIT_CLOSEDLOOP_ACTIVE" in the channel state 
    // can be monitored to determine the end of the movement.
    exitOnError(result);
}



void move(int8_t channel, int32_t moveMode, int8_t direction) {
    SA_CTL_Result_t result;
    int64_t moveValue;
    // Set move mode depending properties for the next movement. 
    switch (moveMode) {
        case SA_CTL_MOVE_MODE_CL_ABSOLUTE:
            // Set move velocity [in pm/s].
            result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 1000000000);
            exitOnError(result);
            // Set move acceleration [in pm/s2].
            result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 1000000000);
            exitOnError(result);
            // Specify absolute position [in pm].
            if (direction) moveValue = -2000000000;
            else moveValue = 1000000000;
            //printf("MCS2 move channel %d to absolute position: %lld pm.\n", channel, moveValue);
            break;
        case SA_CTL_MOVE_MODE_CL_RELATIVE:
            // Set move velocity [in pm/s].
            result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_VELOCITY, 500000000);
            exitOnError(result);
            // Set move acceleration [in pm/s2], a value of 0 disables the acceleration control.
            result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_MOVE_ACCELERATION, 0);
            exitOnError(result);
            // Specify relative position distance [in pm] and direction.
            moveValue = 500000000;
            if (direction) moveValue = -moveValue;
            printf("MCS2 move channel %d relative: %lld pm.\n", channel, moveValue);
            break;
        case SA_CTL_MOVE_MODE_SCAN_ABSOLUTE:
            // Set scan velocity [in dac increments/s].
            // Valid range: 1 to 65535000000
            result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_SCAN_VELOCITY, (65535*2));
            exitOnError(result);
            // Specify absolute scan target to which to scan to [in dac increments].
            // Valid range: 0 to 65535 corresponding to 0 to 100V piezo voltage
            if (direction) moveValue = 0;
            else moveValue = 65535;
            printf("MCS2 scan channel %d absolute to: %lld.\n", channel, moveValue);
            break;
        case SA_CTL_MOVE_MODE_SCAN_RELATIVE:
            // Set scan velocity [in dac increments/s].
            result = SA_CTL_SetProperty_i64(dHandle, channel, SA_CTL_PKEY_SCAN_VELOCITY, 65535);
            exitOnError(result);
            // Specify relative scan target and direction to which to scan to [in dac increments].
            // Valid range: -65535 to 65535 corresponding to 0 to 100V piezo voltage
            // If the resulting absolute scan target exceeds the valid range the scan movement will stop at the boundary.
            moveValue = 65535;
            if (direction) moveValue = -moveValue;
            printf("MCS2 scan channel %d relative: %lld.\n", channel, moveValue);
            break;
        case SA_CTL_MOVE_MODE_STEP:
            // Set step frequency [in Hz].
            // Valid range: 1 to 20000 Hz
            result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_STEP_FREQUENCY, 1000);
            exitOnError(result);
            // Set maximum step amplitude [in dac increments].
            // valid range: 0 to 65535 corresponding to 0 to 100V piezo voltage
            // Lower amplitude values result in smaller step width.
            result = SA_CTL_SetProperty_i32(dHandle, channel, SA_CTL_PKEY_STEP_AMPLITUDE, 65535);
            exitOnError(result);
            // Specify the number of steps to perform and the direction.
            moveValue = 500;
            if (direction) moveValue = -moveValue;
            printf("MCS2 open loop step move, channel %d, steps: %lld.\n", channel, moveValue);
            break;
    }
    // Start actual movement.
    result = SA_CTL_Move(dHandle, channel, moveValue, 0);
    // Note that the function call returns immediatelly, without waiting for the movement to complete.
    // The "CH_STATE_BIT_ACTIVELY_MOVING" and "CH_STATE_BIT_CLOSEDLOOP_ACTIVE" in the channel state 
    // can be monitored to determine the end of the movement.
    exitOnError(result);
}

// STOP
// This command stops any ongoing movement. It also stops the hold position feature of a closed loop command. 
// Note for closed loop movements with acceleration control enabled:
// The first "stop" command sent while moving triggers the positioner to come to a halt by decelerating to zero.
// A second "stop" command triggers a hard stop ("emergency stop").
/*
void stop(int8_t channel) {
    SA_CTL_Result_t result;
    printf("MCS2 stop channel: %d.\n", channel);
    result = SA_CTL_Stop(dHandle, channel, 0);
    //exitOnError(result);
}
*/
void stop(int8_t channel) {
    SA_CTL_Result_t result;
    printf("MCS2 stop channel: %d.\n", channel);
    result = SA_CTL_Stop(dHandle, channel, 0);
    exitOnError(result);
}
