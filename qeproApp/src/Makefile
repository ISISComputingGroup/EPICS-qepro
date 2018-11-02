TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Build the IOC support library

USR_INCLUDES += -I$(SEABREEZE)/vendor/SeaBreeze/include

LIBRARY_IOC += qepro
qepro_SRCS += drvUSBQEPro.cpp drvUSBQEProOBP.cpp

qepro_LIBS += seabreeze asyn
qepro_LIBS_WIN32 += oncrpc
qepro_LIBS += $(EPICS_BASE_IOC_LIBS)


DBD += drvUSBQEProSupport.dbd

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
