include $(EPICS_ENV_PATH)/module.Makefile

EXCLUDE_ARCHS = eldk

STARTUPS = startup/qepro.cmd
DOC      = doc
OPIS     = opi

DBDS = src/drvUSBQEProSupport.dbd

AUTO_DEPENDENCIES = NO
USR_DEPENDENCIES += asyn,4.3+
USR_DEPENDENCIES += seabreeze,3.0.11-ESS0

#USR_CPPFLAGS += -I/vagrant/SeaBreeze/include -DLINUX

#SEABREEZE_DIR = $(abspath ../../src/seabreeze/lib)
#USR_LDFLAGS += -L$(SEABREEZE_DIR) -lseabreeze
USR_LDFLAGS += -lusb

