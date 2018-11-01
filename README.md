# m-epics-qeproasyn

EPICS support for Ocean Optics QEPro spectrometer

## Dependencies

This driver uses the Ocean Optics seabreeze open-source library.

## USB configuration

USB permissions must be modified. Create ``/etc/udev/rules.d/50-usb.rules``, with the following line in it:

	SUBSYSTEMS=="usb", ATTRS{idVendor}=="2457",ATTRS{idProduct}=="4004",GROUP="root",MODE="0666"

This will mount the QEPro with read/write permission for owner, group, and other users.


