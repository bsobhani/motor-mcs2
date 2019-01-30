# Makefile
TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library
LIBRARY_IOC  = smarActMCS2Motor
#LIBRARY_IOC += aaaqq

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += devSmarActMCS2Motor.dbd

INC += smarActMCS2MotorDriver.h
INC += smarActMCS2TriggerDriver.h

USR_CXXFLAGS += -std=c++0x
smarActMCS2Motor_CXXFLAGS += -w

# The following are compiled and added to the Support library
smarActMCS2Motor_SRCS += MCS2Example_Movement.cpp 
smarActMCS2Motor_SRCS += smarActMCS2MotorDriver.cpp
smarActMCS2Motor_SRCS += smarActMCS2TriggerDriver.cpp
#smarActMCSMotor_SRCS += MCS2Example_Movement.c 

smarActMCS2Motor_LIBS += motor
smarActMCS2Motor_LIBS += asyn
smarActMCS2Motor_SYS_LIBS += smaractctl 
smarActMCS2Motor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

