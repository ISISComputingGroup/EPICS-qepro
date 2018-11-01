#@field PREFIX
#@type STRING

#@field PORT
#@type STRING

#@field SIZE
#@type INTEGER

#@field LASER
#@type INTEGER

#@field EPICS_CA_MAX_ARRAY_BYTES
#@type INTEGER

require qeproasyn,0.0.4+
require busy,1.6.0+
require autosave,5.7.0
require seabreeze,3.0.11-ESS0

#epicsEnvSet(EPICS_CA_ADDR_LIST,10.0.2.15)
#epicsEnvSet(EPICS_CA_MAX_ARRAY_BYTES,64000000)

#epicsEnvSet("PREFIX", "Sp1")
#epicsEnvSet("PORT",   "QEPro")
#epicsEnvSet("SIZE",   "3648")
#epicsEnvSet("LASER",  "522")

drvUSBQEProConfigure("$(PORT)","$(SIZE)","$(LASER)")

asynSetTraceMask("$(PORT)", -1, 0x0)
#asynSetTraceMask("$(PORT)", -1, 0x1)
#asynSetTraceMask("$(PORT)", -1, 0x9)
#asynSetTraceMask("$(PORT)", -1, 0xF)
#asynSetTraceMask("$(PORT)", -1, 0x11)
#asynSetTraceMask("$(PORT)", -1, 0xFF)
asynSetTraceIOMask("$(PORT)", -1, 0x0)
#asynSetTraceIOMask("$(PORT)", -1, 0x2)

dbLoadRecords(qepro.template, "PREFIX=$(PREFIX), PORT=$(PORT), ADDR=0, TIMEOUT=1, SIZE=$(SIZE), LASER=$(LASER)")

## autosave/restore machinery
save_restoreSet_Debug(0)
save_restoreSet_IncompleteSetsOk(1)
save_restoreSet_DatedBackupFiles(1)
 
set_savefile_path("autosave","/sav")
set_requestfile_path("autosave","/req")
 
#set_pass0_restoreFile("info_positions.sav")
set_pass0_restoreFile("info_settings.sav")
set_pass1_restoreFile("info_settings.sav")
 
iocInit()
 
## more autosave/restore machinery
cd autosave/req
makeAutosaveFiles()
cd ../..
create_monitor_set("info_positions.req", 5 , "")
create_monitor_set("info_settings.req", 15 , "")

