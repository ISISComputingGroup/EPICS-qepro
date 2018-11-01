#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <iocsh.h>
#include <epicsExport.h>
#include <alarm.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>
#include <sys/stat.h>

#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <vector>

#include "drvUSBQEPro.h"
#include "drvUSBQEProWorker.h"
#include "api/SeaBreezeWrapper.h"

#define NUM_QEPRO_PARAMS ((int)(&LAST_QEPRO_PARAM - &FIRST_QEPRO_PARAM + 1))

static const char *driverName="drvUSBQEPro";

drvUSBQEPro::drvUSBQEPro(const char *portName, int maxPoints, double laser)
   : asynPortDriver(portName,
                    1, /* maxAddr */
                    (int)NUM_QEPRO_PARAMS,
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynDrvUserMask | asynOctetMask, /* Interface mask */
                    asynInt32Mask | asynFloat64Mask | asynFloat64ArrayMask | asynOctetMask,  /* Interrupt mask */
                    0, /* asynFlags.  This driver does not block and it is not multi-device, so flag is 0 */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0), /* Default stack size*/
    m_laser(laser),
    m_poll_time(POLL_TIME)
{
    spec_index = 0;

    // Initialize connection status
    connected = false;
    ioc_starting = true;

    //eventId = epicsEventCreate(epicsEventEmpty);
    createParam( QEProNumSpecs,             asynParamInt32,         &P_numSpecs);
    createParam( QEProId,                   asynParamInt32,         &P_nrBoard);
    createParam( QEProName,                 asynParamOctet,         &P_name);
    createParam( QEProFirmwareVersion,      asynParamOctet,         &P_firmwareVersion);
    createParam( QEProFirmwareModel,        asynParamOctet,         &P_firmwareModel);
    createParam( QEProSerialNumber,         asynParamOctet,         &P_serialNumber);
    createParam( QEProNumberOfPixels,       asynParamInt32,         &P_numberOfPixels);
    createParam( QEProNumberOfDarkPixels,   asynParamInt32,         &P_numberOfDarkPixels);
    createParam( QEProIntegrationTime,      asynParamInt32,         &P_integrationTime);
    createParam( QEProMaxIntegrationTime,   asynParamInt32,         &P_maxIntegrationTime);
    createParam( QEProMinIntegrationTime,   asynParamInt32,         &P_minIntegrationTime);
    createParam( QEProMaxIntensity,  	    asynParamFloat64,       &P_maxIntensity);
    createParam( QEProBoxcarWidth,  	    asynParamInt32,         &P_boxcarWidth);
    createParam( QEProElectricDark,  	    asynParamInt32,         &P_electricDark);
    createParam( QEProDetectorTemperature,  asynParamInt32,         &P_detectorTemperature);
    createParam( QEProBoardTemperature,     asynParamInt32,         &P_boardTemperature);
    createParam( QEProTempSetPoint,         asynParamInt32,         &P_tempSetPoint);
    createParam( QEProTriggerMode,          asynParamInt32,         &P_triggerMode);
    createParam( QEProNonLinearity,	        asynParamInt32,         &P_nonLinearity);
    createParam( QEProDecouple,             asynParamInt32,         &P_decouple);
    createParam( QEProLEDIndicator,         asynParamInt32,         &P_ledIndicator);
    createParam( QEProAverages,             asynParamInt32,         &P_averages);
    createParam( QEProXAxisNm,              asynParamFloat64Array,  &P_xAxisNm);
    createParam( QEProXAxisRs,              asynParamFloat64Array,  &P_xAxisRs);
    createParam( QEProSpectrum,             asynParamFloat64Array,  &P_spectrum);
    createParam( QEProLaser,                asynParamFloat64,       &P_laser);
    createParam( QEProConnected,            asynParamInt32,         &P_connected);
    createParam( QEProAcqMode,              asynParamInt32,         &P_acqMode);
    createParam( QEProAcqStart,             asynParamInt32,         &P_acqStart);
    createParam( QEProAcqStop,              asynParamInt32,         &P_acqStop);
    createParam( QEProAcqSts,               asynParamInt32,         &P_acqSts);
    createParam( QEProFileWrite,            asynParamInt32,         &P_fileWrite);
    createParam( QEProFilePath,             asynParamOctet,         &P_filePath);
    createParam( QEProFileName,             asynParamOctet,         &P_fileName);
    createParam( QEProFullFileName,         asynParamOctet,         &P_fullFileName);
    createParam( QEProFullFilePath,         asynParamOctet,         &P_fullFilePath);
    createParam( QEProFileIndex,            asynParamInt32,         &P_fileIndex);
    createParam( QEProXAxisMode,            asynParamInt32,         &P_xAxisMode);
    createParam( QEProXAxis,                asynParamFloat64Array,  &P_xAxis);
    createParam( QEProCPUTemperature,       asynParamFloat64,       &P_cpuTemperature);
    createParam( QEProPCBTemperature,       asynParamFloat64,       &P_pcbTemperature);
    createParam( QEProDetTemperature,       asynParamFloat64,       &P_detTemperature);
    createParam( QEProROI0LowWavelength,    asynParamFloat64,       &P_roi0LowWavelength);
    createParam( QEProROI0HighWavelength,   asynParamFloat64,       &P_roi0HighWavelength);
    createParam( QEProROI1LowWavelength,    asynParamFloat64,       &P_roi1LowWavelength);
    createParam( QEProROI1HighWavelength,   asynParamFloat64,       &P_roi1HighWavelength);
    createParam( QEProROI0Sum,              asynParamFloat64,       &P_roi0Sum);
    createParam( QEProROI1Sum,              asynParamFloat64,       &P_roi1Sum);
    createParam( QEProROI0Fraction,         asynParamFloat64,       &P_roi0Fraction);
    createParam( QEProROI1Fraction,         asynParamFloat64,       &P_roi1Fraction);
    createParam( QEProDarkAcq,              asynParamInt32,         &P_darkAcq);
    createParam( QEProDarkSubtract,         asynParamInt32,         &P_darkSubtract);
    createParam( QEProDarkSpectrum,         asynParamFloat64Array,  &P_darkSpectrum);
    createParam( QEProDarkValid,            asynParamInt32,         &P_darkValid);
    createParam( QEProDarkValidOverride,    asynParamInt32,         &P_darkValidOverride);
    createParam( QEProBGAcq,                asynParamInt32,         &P_bgAcq);
    createParam( QEProBGSubtract,           asynParamInt32,         &P_bgSubtract);
    createParam( QEProBGSpectrum,           asynParamFloat64Array,  &P_bgSpectrum);
    createParam( QEProBGValid,              asynParamInt32,         &P_bgValid);
    createParam( QEProBGValidOverride,      asynParamInt32,         &P_bgValidOverride);

    // Set up initial USB context. Must be done before starting thread,
    // or attempting comms to device.
    int rc = 0;
    context = NULL;
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "drvUSBQEPro: init usb\n");
    rc = libusb_init(&context);
    assert(rc == 0);

    // General initialisation
    dark_valid = false;
    dark_valid_override = false;
    bg_valid = false;
    bg_valid_override = false;

    // Initialise the file type strings
    file_extensions.push_back("");
    file_extensions.push_back("_raw");
    file_extensions.push_back("_bg");
    file_extensions.push_back("_raw_bg");
    file_extensions.push_back("_dark");

    file_descriptions.push_back("Processed data");
    file_descriptions.push_back("Raw data");
    file_descriptions.push_back("Processed background data");
    file_descriptions.push_back("Raw background data");
    file_descriptions.push_back("Dark spectrum");

    asynPrint(pasynUserSelf,
            ASYN_TRACE_FLOW,
            "drvUSBQEPro: create spectrum readout thread\n");

    epicsThreadCreate("drvUSBQEProThread",
            epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC)worker,
            this);
}
//-----------------------------------------------------------------------------------------------------------------

void drvUSBQEPro::getSpectrumThread(void *priv){

    int error;
    int acq_mode;
    int start;
    int stop;
    bool run = false;

    while(1) {

        // Get acquisition mode and control status from PVs
        getIntegerParam(P_acqMode, &acq_mode);
        getIntegerParam(P_acqStart, &start);
        getIntegerParam(P_acqStop, &stop);

        asynPrint(pasynUserSelf,
                ASYN_TRACE_FLOW,
                "getSpectrumThread: acquisition mode = %d\n",
                acq_mode);
        asynPrint(pasynUserSelf,
                ASYN_TRACE_FLOW,
                "getSpectrumThread: device_id = 0x%lx\n",
                device_id);
        asynPrint(pasynUserSelf,
                ASYN_TRACE_FLOW,
                "getSpectrumThread: spectrometer_feature_id = 0x%lx\n",
                spectrometer_feature_id);
        asynPrint(pasynUserSelf,
                ASYN_TRACE_FLOW,
                "getSpectrumThread: num_pixels = %d\n",
                num_pixels);

        if (start &&
                (acq_mode == QEPRO_ACQ_MODE_SINGLE ||
                 acq_mode == QEPRO_ACQ_MODE_CONTINUOUS )) {
            run = true;
        }
        else if (start &&
                acq_mode == QEPRO_ACQ_MODE_OFF) {
            run = false;
        }
        else if (stop) {
            run = false;
        }

        // Collect a dark spectra if requested
        if (dark_acquire) {
            acquire_dark();
        }

        // Collect a background spectra if requested
        if (bg_acquire) {
            acquire_bg();
        }

        // Decide if we should acquire or not
        if (run) {
            test_connection();
            if (connected) {
                lock();

                // If not yet acquiring, start the detector
                if (!acquiring)
                    start_acquisition();

                // Set acquisition status PV
                acquiring = true;
                setIntegerParam(P_acqSts, acquiring);
                callParamCallbacks();

                // NOTE: This function blocks until a new spectrum is available
                api->spectrometerGetFormattedSpectrum(
                        device_id,
                        spectrometer_feature_id,
                        &error,
                        raw_spectrum_buffer,
                        num_pixels);

                unlock();

                update_data_spectrum();

                integrate_rois();

                update_axis_arrays();

                write_data_files();

            }
        }
        else {
            // Stop the acquisition
            test_connection();
            if (acquiring && connected) {
                abort();
                clear_buffers();
            }
            // Set acquisition status PV
            acquiring = false;
            setIntegerParam(P_acqSts, acquiring);
            callParamCallbacks();
        }

        // Do a single acquisition
        if (acq_mode == QEPRO_ACQ_MODE_SINGLE) {
            run = false;
        }
        //TODO: Check if we actually need this
        epicsThreadSleep(m_poll_time);
    }
}

void drvUSBQEPro::get_wavelengths() {
    int error;
    num_wavelengths = api->spectrometerGetWavelengths(
            device_id,
            spectrometer_feature_id,
            &error,
            wavelength_buffer,
            num_pixels);

    convert_nm_to_raman_shift(
            raman_shift_buffer,
            wavelength_buffer);

    update_axis_arrays();
}

void drvUSBQEPro::update_axis_arrays() {
    int x_axis_mode;
    // Do all the other array callbacks
    getIntegerParam(P_xAxisMode, &x_axis_mode);
    // Send the values to the PV used for the plot x axis
    if (x_axis_mode == QEPRO_XAXIS_RAMAN_SHIFT) {
        doCallbacksFloat64Array(
                raman_shift_buffer,
                num_wavelengths,
                P_xAxis,
                0);
    }
    else {
        doCallbacksFloat64Array(
                wavelength_buffer,
                num_wavelengths,
                P_xAxis,
                0);
    }

    doCallbacksFloat64Array(
            raman_shift_buffer,
            num_wavelengths,
            P_xAxisRs,
            0);

    doCallbacksFloat64Array(
            wavelength_buffer,
            num_wavelengths,
            P_xAxisNm,
            0);
}

void drvUSBQEPro::acquire_dark() {
    test_connection();
    if (connected) {
        int error;

        asynPrint(pasynUserSelf,
                ASYN_TRACE_FLOW,
                "getSpectrumThread: acquiring dark spectrum\n");

        // Restart acquisition
        abort();
        clear_buffers();
        start_acquisition();

        api->spectrometerGetFormattedSpectrum(
                device_id,
                spectrometer_feature_id,
                &error,
                dark_buffer,
                num_pixels);

        get_wavelengths();

        // Stop the acquisition after readout
        abort();
        clear_buffers();

        dark_valid = true;
        setIntegerParam(P_darkValid, dark_valid);
        callParamCallbacks();
        dark_acquire = false;

        // Send the data to the dark spectrum PV
        doCallbacksFloat64Array(
                dark_buffer,
                num_pixels,
                P_darkSpectrum,
                0);
    }
}

void drvUSBQEPro::acquire_bg() {
    test_connection();
    if (connected) {
        int error;

        asynPrint(pasynUserSelf,
                ASYN_TRACE_FLOW,
                "getSpectrumThread: acquiring background spectrum\n");

        // Restart acquisition
        abort();
        clear_buffers();
        start_acquisition();

        api->spectrometerGetFormattedSpectrum(
                device_id,
                spectrometer_feature_id,
                &error,
                raw_bg_buffer,
                num_pixels);

        get_wavelengths();

        // Stop the acquisition after readout
        abort();
        clear_buffers();

        bg_valid = true;
        setIntegerParam(P_bgValid, bg_valid);
        callParamCallbacks();
        bg_acquire = false;

        update_bg();
    }
}

void drvUSBQEPro::update_data_spectrum() {
    int boxcar_half_width;

    getIntegerParam(P_boxcarWidth, &boxcar_half_width);

    asynPrint(pasynUserSelf,
            ASYN_TRACE_FLOW,
            "getSpectrumThread: boxcar_half_width = %d\n",
            boxcar_half_width);

    if (bg_subtract && (bg_valid || bg_valid_override)) {
        asynPrint(
                pasynUserSelf,
                ASYN_TRACE_FLOW,
                "update_data_spectrum: doing background subtraction\n");

        subtract_spectra(spectrum_buffer, raw_spectrum_buffer, raw_bg_buffer);
    }
    else if (dark_subtract && (dark_valid || dark_valid_override)) {
        asynPrint(
                pasynUserSelf,
                ASYN_TRACE_FLOW,
                "update_data_spectrum: doing dark subtraction\n");

        subtract_spectra(spectrum_buffer, raw_spectrum_buffer, dark_buffer);
    }
    else {
        memcpy(
                spectrum_buffer,
                raw_spectrum_buffer,
                num_pixels * sizeof(double));
    }

    if (boxcar_half_width > 0) {
        boxcar(spectrum_buffer,
                spectrum_buffer,
                boxcar_half_width);
    }

    // Send the data to the background spectrum PV
    doCallbacksFloat64Array(
            spectrum_buffer,
            num_pixels,
            P_spectrum,
            0);
}


void drvUSBQEPro::update_bg() {
    if (dark_subtract && (dark_valid || dark_valid_override)) {
        asynPrint(
                pasynUserSelf,
                ASYN_TRACE_FLOW,
                "getSpectrumThread: doing dark subtraction\n");

        subtract_spectra(bg_buffer, raw_bg_buffer, dark_buffer);
    }
    else {
        memcpy(
                bg_buffer,
                raw_bg_buffer,
                num_pixels * sizeof(double));
    }

    // Send the data to the background spectrum PV
    doCallbacksFloat64Array(
            bg_buffer,
            num_pixels,
            P_bgSpectrum,
            0);
}

void drvUSBQEPro::subtract_spectra(
        double *result_spectrum,
        const double *spectrum1,
        const double *spectrum2) {
    for (int i = 0; i < num_pixels; i++) {
        result_spectrum[i] = spectrum1[i] - spectrum2[i];
        if (result_spectrum[i] < 0)
            result_spectrum[i] = 0;
    }
}

void drvUSBQEPro::integrate_rois() {
    for (int i = 0; i < NUM_ROIS; i++) {
        roi_sum[i] = 0;
        for (int j = 0; j < num_pixels; j++) {
            if (wavelength_buffer[j] > roi_low[i]
                    && wavelength_buffer[j] < roi_high[i]) {
                roi_sum[i] += spectrum_buffer[j];
            }
        }
    }

    for (int i = 0; i < NUM_ROIS; i++) {
        roi_fraction[i] = roi_sum[i] / (roi_sum[0] + roi_sum[1]);
    }

    setDoubleParam(P_roi0Fraction, roi_fraction[0]);
    setDoubleParam(P_roi1Fraction, roi_fraction[1]);
    setDoubleParam(P_roi0Sum, roi_sum[0]);
    setDoubleParam(P_roi1Sum, roi_sum[1]);
    callParamCallbacks();
}

void drvUSBQEPro::convert_nm_to_raman_shift(
        double *raman_shift_buffer,
        const double *wavelength_buffer) {
    // Calculate Raman shift in cm-1
    for(int i = 0; i < num_wavelengths; i++)
        raman_shift_buffer[i] =
            (1./m_laser - 1./wavelength_buffer[i]) *10e7;
}

void drvUSBQEPro::allocate_spectrum_buffer() {
    int error;
    if (connected) {
        num_pixels = api->spectrometerGetFormattedSpectrumLength(
                device_id,
                spectrometer_feature_id,
                &error);
        raw_spectrum_buffer = (double *)calloc(num_pixels, sizeof(double));
        spectrum_buffer = (double *)calloc(num_pixels, sizeof(double));
        wavelength_buffer = (double *)calloc(num_pixels, sizeof(double));
        raman_shift_buffer = (double *)calloc(num_pixels, sizeof(double));
        dark_buffer = (double *)calloc(num_pixels, sizeof(double));
        raw_bg_buffer = (double *)calloc(num_pixels, sizeof(double));
        bg_buffer = (double *)calloc(num_pixels, sizeof(double));
    }
}

void drvUSBQEPro::deallocate_spectrum_buffer() {
    if (!connected) {
        if (raw_spectrum_buffer)
            free(raw_spectrum_buffer);
        raw_spectrum_buffer = NULL;

        if (spectrum_buffer)
            free(spectrum_buffer);
        spectrum_buffer = NULL;

        if (wavelength_buffer)
            free(wavelength_buffer);
        wavelength_buffer = NULL;

        if (raman_shift_buffer)
            free(raman_shift_buffer);
        raman_shift_buffer = NULL;

        if (dark_buffer)
            free(dark_buffer);
        dark_buffer = NULL;

        if (raw_bg_buffer)
            free(raw_bg_buffer);
        raw_bg_buffer = NULL;

        if (bg_buffer)
            free(bg_buffer);
        bg_buffer = NULL;
    }
}

//--------------------------------------------------------------------------------------------
// No specific function needed for writeOctet. Use base class implementation.
// asynStatus drvUSBQEPro::writeOctet (asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason){
// }

asynStatus drvUSBQEPro::readOctet (asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason){

    int addr=0;
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;

    this->getAddress(pasynUser, &addr);

    char buffer[80];
    int error;

    test_connection();
    if (connected) {
        if (function == P_name) {
            api->getDeviceType(device_id, &error, buffer, 79);
            strcpy(value, buffer);
            *nActual = strlen(buffer);
            *eomReason = 0;
            setStringParam(P_name, buffer);
        }
        else if (function == P_firmwareVersion) {
            strcpy(buffer, "Not available");
            strcpy(value, buffer);
            *nActual = strlen(buffer);
            *eomReason = 0;
            setStringParam(P_firmwareVersion, buffer);
        }
        else if (function == P_firmwareModel) {
            strcpy(buffer, "Not available");
            strcpy(value, buffer);
            *nActual = strlen(buffer);
            *eomReason = 0;
            setStringParam(P_firmwareModel, buffer);
        }
        else if (function == P_serialNumber) {
            api->getSerialNumber(
                    device_id,
                    serial_number_feature_id,
                    &error,
                    buffer,
                    79);
            strcpy(value, buffer);
            *nActual = strlen(buffer);
            *eomReason = 0;
            setStringParam(P_serialNumber, buffer);
        }
        else {
            // All other parameters just get set in parameter list, no need to
            //  act on them here
        }
    }
    else {
        // Use this status return if the device is not connected
        status = asynDisconnected;
    }

    callParamCallbacks(addr);
    return status;
}
//--------------------------------------------------------------------------------------------
asynStatus drvUSBQEPro::readInt32 (asynUser *pasynUser, epicsInt32 *value){

    int addr;
    int function;
    asynStatus status = asynSuccess;
    int rval;
    const char* functionName = "readInt32";
    int error;

    function = pasynUser->reason;
    this->getAddress(pasynUser, &addr);

    // Handle all device related parameters. Must test connection first
    // before attempting to read.
    test_connection();
    if (connected) {
        if (function == P_numSpecs) {
            rval = api->getNumberOfDeviceIDs();
            *value = rval;
            setIntegerParam(addr, P_numSpecs, *value);
        }
        else if (function == P_numberOfPixels) {
            rval = api->spectrometerGetFormattedSpectrumLength(
                    device_id,
                    spectrometer_feature_id,
                    &error);
            *value = rval;
            setIntegerParam(addr, P_numberOfPixels, *value);
            num_pixels = rval;
        }
        else if (function == P_numberOfDarkPixels) {
            rval = api->spectrometerGetElectricDarkPixelCount(
                    device_id,
                    spectrometer_feature_id,
                    &error);
            *value = rval;
            setIntegerParam(addr, P_numberOfDarkPixels, *value);
        }
        else if (function == P_integrationTime) {
            rval = integration_time;
            // function return in microseconds, we want to have in miliseconds
            rval /= 1000;
            *value = rval;
            setIntegerParam(addr, P_integrationTime, *value);
        }
        else if (function == P_maxIntegrationTime) {
            rval = api->spectrometerGetMaximumIntegrationTimeMicros(
                    device_id,
                    spectrometer_feature_id,
                    &error);
            // function return in microseconds, we want to have in miliseconds
            rval /= 1000;
            *value = rval;
            setIntegerParam(addr, P_maxIntegrationTime, *value);
        }
        else if (function == P_minIntegrationTime) {
            rval = api->spectrometerGetMinimumIntegrationTimeMicros(
                    device_id,
                    spectrometer_feature_id,
                    &error);
            // function return in microseconds, we want to have in miliseconds
            rval /= 1000;
            *value = rval;
            setIntegerParam(addr, P_minIntegrationTime, *value);
        }
        else if (function == P_triggerMode) {
            rval = trigger_mode;
            *value = rval;
            setIntegerParam(addr, P_triggerMode, *value);
        }

        else if (function == P_averages) {
            // Feature not yet implemented in QEPro seabreeze driver
        }
        else if (function == P_decouple) {
            // Feature not implemented in QEPro?
            *value = 0;
            setIntegerParam(addr, P_triggerMode, *value);
        }
        else if (function == P_ledIndicator) {
            *value = 0;
            setIntegerParam(addr, P_ledIndicator, *value);
        }
        else {
            status = asynPortDriver::readInt32(pasynUser, value);
        }
    }
    else {
        status = asynDisconnected;
    }

    // Handle parameters that are not read from the device
    if (function == P_boxcarWidth) {
        getIntegerParam(P_boxcarWidth, &rval);
        *value = rval;
        status = asynSuccess;
    }
    else if (function == P_connected) {
        *value = connected;
        status = asynSuccess;
    }
    else if (function == P_averages) {
        getIntegerParam(P_averages, &rval);
        *value = rval;
        status = asynSuccess;
    }

    callParamCallbacks(addr);

    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: port=%s, value=%d, addr=%d, status=%d\n",
            driverName, functionName, this->portName, *value, addr, (int)status);

    return status;
}

//--------------------------------------------------------------------------------------------

asynStatus drvUSBQEPro::readFloat64(asynUser *pasynUser, epicsFloat64 *value){
    int addr;
    int function;
    asynStatus status = asynSuccess;
    double rval;
    const char* functionName = "readFloat64";
    int error;

    function = pasynUser->reason;
    this->getAddress(pasynUser, &addr);

    test_connection();
    if (connected) {
        if (function == P_maxIntensity) {
            rval = api->spectrometerGetMaximumIntensity(
                    device_id,
                    spectrometer_feature_id,
                    &error);
            *value = rval;
            setDoubleParam(addr, P_maxIntensity, *value);
        }
        else if (function == P_nonLinearity) {
            double buffer;
            api->nonlinearityCoeffsGet(
                    device_id,
                    nonlinearity_feature_id,
                    &error,
                    &buffer,
                    1);
            *value = buffer;
            setDoubleParam(addr, P_nonLinearity, *value);
        }
        else if (function == P_cpuTemperature) {
            read_temperatures();
            *value = temperatures[CPU_TEMPERATURE];
        }
        else if (function == P_pcbTemperature) {
            *value = temperatures[PCB_TEMPERATURE];
        }
        else if (function == P_detTemperature) {
            *value = temperatures[TEC_TEMPERATURE];
        }
        else {
            status = asynPortDriver::readFloat64(pasynUser, value);
        }
    }
    else
        status = asynDisconnected;

    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: port=%s, value=%f, addr=%d, status=%d\n",
            driverName, functionName, this->portName, *value, addr, (int)status);

    return status;
}

//asynStatus drvUSBQEPro::readFloat64Array(asynUser *pasynUser, epicsInt32 value)
// Not required. All asynParamFloat64Array data is handled by array callbacks.

//--------------------------------------------------------------------------------------------

asynStatus drvUSBQEPro::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *paramName;
    const char* functionName = "writeInt32";
    int addr;
    int error;
    int triggerMode;
    int decouple, ledIndicator;

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setIntegerParam(function, value);

    /* Fetch the parameter string name for possible use in debugging */
    getParamName(function, &paramName);

    this->getAddress(pasynUser, &addr);
    //    getIntegerParam(addr, nrBoard, &index);

    test_connection();
    if (connected) {
        if (function == P_integrationTime) {
            set_integration_time();
        }
        else if (function == P_averages) {
            // Check if implmemented in QEPro
        }
        else if (function == P_boxcarWidth) {
            // Update the calculated spectrum
            update_data_spectrum();
        }
        else if (function == P_electricDark) {
            // Check if implmemented in QEPro
        }
        else if (function == P_nonLinearity) {
            // Check if implmemented in QEPro
        }
        else if (function == P_triggerMode) {
            getIntegerParam(P_triggerMode, &triggerMode);
            api->spectrometerSetTriggerMode(
                    device_id,
                    spectrometer_feature_id,
                    &error,
                    triggerMode);
        }
        //    in this version setPoint is not supported....
        //    other parameters for controling temperature and fan also not supported....
        //    else if (function == setPonit) {
        //        ThermoElectricWrapper thermoElectric = getFeatureControllerThermoElectric(index);
        //        if(thermoElectric)
        //           getIntegerParam(addr, setPoint, &setPoint);
        //           thermoElectric.setTECEnable(true);
        //           thermoElectric.setDetectorSetPointCelsius(temp);
        //        } else return statusError;
        else if (function == decouple) {
            // Check if implmemented in QEPro
        }
        else if (function == ledIndicator) {
            // Check if implmemented in QEPro
        }

        else {
            /* All other parameters just get set in parameter list, no need to
             *          * act on them here */
        }
    }

    // Do functions that are not dependent on connection
    if (function == P_darkSubtract) {
        if (value == 0)
            dark_subtract = false;
        else
            dark_subtract = true;
        update_bg();
        update_data_spectrum();
    }
    else if (function == P_darkAcq) {
        if (value == 1)
            dark_acquire = true;
    }
    else if (function == P_darkValidOverride) {
        if (value == 1)
            dark_valid_override = true;
        else
            dark_valid_override = false;
    }
    else if (function == P_bgSubtract) {
        if (value == 0)
            bg_subtract = false;
        else
            bg_subtract = true;
        update_data_spectrum();
    }
    else if (function == P_bgAcq) {
        if (value == 1)
            bg_acquire = true;
    }
    else if (function == P_bgValidOverride) {
        if (value == 1)
            bg_valid_override = true;
        else
            bg_valid_override = false;
    }

    /* Do callbacks so higher layers see any changes */
    status = (asynStatus) callParamCallbacks();

    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: port=%s, value=%d, addr=%d, status=%d\n",
            driverName, functionName, this->portName, value, addr, (int)status);
    return status;
}

asynStatus drvUSBQEPro::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char* functionName = "writeFloat64";
    int addr;

    /* Set the parameter in the parameter library. */
    status = (asynStatus) setDoubleParam(function, value);

    this->getAddress(pasynUser, &addr);

    if (function == P_roi0LowWavelength) {
        roi_low[0] = value;
        integrate_rois();
    }
    else if (function == P_roi0HighWavelength) {
        roi_high[0] = value;
        integrate_rois();
    }
    else if (function == P_roi1LowWavelength) {
        roi_low[1] = value;
        integrate_rois();
    }
    else if (function == P_roi1HighWavelength) {
        roi_high[1] = value;
        integrate_rois();
    }

    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, "%s:%s: port=%s, value=%f, addr=%d, status=%d\n",
            driverName, functionName, this->portName, value, addr, (int)status);

    return status;
}

// Test whether spectrometer is connected. If it is was previously
// disconnected, read the various id values and store to avoid needing to
// read each time.
void drvUSBQEPro::test_connection() {
    int error;
    bool found_ooi_spectrometer = false;

    // Check the USB device list to see if the Ocean Optics spectrometer
    //context = NULL;
    libusb_device **list;
    int rc = 0;
    ssize_t count = 0;

    count = libusb_get_device_list(context, &list);
    assert(count > 0);

    for (ssize_t idx = 0; idx < count; ++idx) {
        libusb_device *device = list[idx];
        libusb_device_descriptor desc = {0};

        rc = libusb_get_device_descriptor(device, &desc);
        assert(rc == 0);

        if (desc.idVendor == drvUSBQEPro::OOI_VENDOR_ID) {
            found_ooi_spectrometer = true;
        }
    }
    libusb_free_device_list(list, 1);

    asynPrint(pasynUserSelf,
            ASYN_TRACE_FLOW,
            "test_connection: found_ooi_spectrometer = %d\n",
            found_ooi_spectrometer);

    // Restart the connection to the spectrometer
    if (found_ooi_spectrometer && !connected) {
        if (api) {
            api->shutdown();
        }

        api = SeaBreezeAPI::getInstance();
        // Add a short delay here to allow the USB connection to work.
        // If this sleep is not present, the IOC will segfault on startup
        // if the spectrometer is already connected to the USB port.
        if (ioc_starting) {
            unsigned int sleep_time = 500000;
            usleep(sleep_time);
            ioc_starting = false;
        }
        api->probeDevices();
    }

    if (!found_ooi_spectrometer) {
        connected = false;
        deallocate_spectrum_buffer();
    }
    else {
        // Check if we were disconnected previously
        if (!connected) {
            // Read device IDs
            int number_of_devices = api->getNumberOfDeviceIDs();
            asynPrint(pasynUserSelf,
                    ASYN_TRACE_FLOW,
                    "test_connection: number of Ocean Optics devices = %d\n",
                    number_of_devices);

            long * device_ids = (long *)calloc(number_of_devices, sizeof(long));
            api->getDeviceIDs(device_ids, number_of_devices);
            // Assume only one device
            device_id = device_ids[0];

            asynPrint(pasynUserSelf,
                    ASYN_TRACE_FLOW,
                    "test_connection: device ID = %ld\n",
                    device_id);

            api->openDevice(device_id, &error);

            asynPrint(pasynUserSelf,
                    ASYN_TRACE_FLOW,
                    "test_connection: opened device ID = %ld. Code = %d [%s]\n",
                    device_id,
                    error,
                    sbapi_get_error_string(error));

            // Read spectrometer feature ID
            int num_spectrometer_features =
                api->getNumberOfSpectrometerFeatures(device_id, &error);

            asynPrint(pasynUserSelf,
                    ASYN_TRACE_FLOW,
                    "test_connection: number of spectrometer features = %d\n",
                    num_spectrometer_features);

            if (num_spectrometer_features > 0) {
                long * spectrometer_feature_ids =
                    (long *)calloc(num_spectrometer_features, sizeof(long));

                api->getSpectrometerFeatures(
                        device_ids[0],
                        &error,
                        spectrometer_feature_ids,
                        num_spectrometer_features);

                spectrometer_feature_id = spectrometer_feature_ids[0];
            }
            asynPrint(pasynUserSelf,
                    ASYN_TRACE_FLOW,
                    "test_connection: spectrometer feature id = 0x%lx\n",
                    spectrometer_feature_id);

            // Read USB feature ID
            int num_usb_features =
                api->getNumberOfRawUSBBusAccessFeatures(device_id, &error);

            asynPrint(pasynUserSelf,
                    ASYN_TRACE_FLOW,
                    "test_connection: number of usb features = %d\n",
                    num_usb_features);

            if (num_usb_features > 0) {
                long * usb_feature_ids =
                    (long *)calloc(num_usb_features, sizeof(long));

                api->getRawUSBBusAccessFeatures(
                        device_ids[0],
                        &error,
                        usb_feature_ids,
                        num_usb_features);

                usb_feature_id = usb_feature_ids[0];
            }
            asynPrint(pasynUserSelf,
                    ASYN_TRACE_FLOW,
                    "test_connection: usb feature id = 0x%lx\n",
                    usb_feature_id);

            // Read serial number feature ID
            int num_serial_number_features =
                api->getNumberOfSerialNumberFeatures(
                        device_id,
                        &error);

            if (num_serial_number_features > 0) {
                long * serial_number_feature_ids =
                    (long *)calloc(
                            num_serial_number_features,
                            sizeof(long));

                api->getSerialNumberFeatures(
                        device_id,
                        &error,
                        serial_number_feature_ids,
                        num_serial_number_features);

                serial_number_feature_id =
                    serial_number_feature_ids[0];
            }


            // Read non-linearity coefficients feature ID
            int num_nonlinearity_features =
                api->getNumberOfNonlinearityCoeffsFeatures(
                        device_id,
                        &error);

            if (num_nonlinearity_features > 0) {
                long * nonlinearity_feature_ids =
                    (long *)calloc(
                            num_nonlinearity_features,
                            sizeof(long));

                api->getNonlinearityCoeffsFeatures(
                        device_id,
                        &error,
                        nonlinearity_feature_ids,
                        num_nonlinearity_features);

                nonlinearity_feature_id =
                    nonlinearity_feature_ids[0];
            }
            connected = true;

            // Perform the functions that only need to be done
            // on establishing connection
            read_number_of_pixels();
            read_serial_number();
            read_device_name();
            // Update PVs
            callParamCallbacks();

            // Allocate memory for spectra
            allocate_spectrum_buffer();

            // Read the wavelength data
            get_wavelengths();

            // Initialize the integration time
            set_integration_time();
        }
    }

    // Update the connected PV status
    setIntegerParam(P_connected, connected);
    callParamCallbacks();
}

void drvUSBQEPro::read_serial_number() {
    int error;
    char *serial_number_buffer;

    size_t len_serial_num =
        api->getSerialNumberMaximumLength(
                device_id,
                serial_number_feature_id,
                &error);

    serial_number_buffer =
        (char *)calloc(len_serial_num, sizeof(char));

    api->getSerialNumber(
                device_id,
                serial_number_feature_id,
                &error,
                serial_number_buffer,
                len_serial_num);

    setStringParam(P_serialNumber, serial_number_buffer);

    if (serial_number_buffer)
        free(serial_number_buffer);
    serial_number_buffer = NULL;

}

void drvUSBQEPro::read_device_name() {
    int error;
    const size_t NAME_SIZE = 40;
    char name_buffer[NAME_SIZE];

    api->getDeviceType(
                device_id,
                &error,
                name_buffer,
                NAME_SIZE);

    setStringParam(P_name, name_buffer);
}

void drvUSBQEPro::set_integration_time() {
    int error;
    int tmp;

    getIntegerParam(P_integrationTime, &tmp);
    integration_time = (unsigned long)tmp;

    api->spectrometerSetIntegrationTimeMicros(
            device_id,
            spectrometer_feature_id,
            &error,
            integration_time);
    // Invalidate the current dark spectra
    dark_valid = false;
    bg_valid = false;
    setIntegerParam(P_integrationTime, integration_time);
    setIntegerParam(P_darkValid, dark_valid);
    setIntegerParam(P_bgValid, bg_valid);
    callParamCallbacks();
}

void drvUSBQEPro::read_number_of_pixels() {
    int error;

    int num_live_pixels =
        api->spectrometerGetFormattedSpectrumLength (
                device_id,
                spectrometer_feature_id,
                &error);

    setIntegerParam(P_numberOfPixels, num_live_pixels);

    // Store in the object
    num_pixels = num_live_pixels;

    int num_dark_pixels =
        api->spectrometerGetElectricDarkPixelCount(
                device_id,
                spectrometer_feature_id,
                &error);

    setIntegerParam(P_numberOfDarkPixels, num_dark_pixels);
}

void drvUSBQEPro::boxcar(
        double *output_buffer,
        const double *input_buffer,
        int boxcar_half_width) {

    int boxcar_width = 2 * boxcar_half_width + 1;
    double sum;
    double average;

    // TODO: Fix algorithm to keep running total. Only need one addition and one
    // subtraction per calculation.
    for (int i = 0; i < num_pixels; i++) {
        sum = 0;
        if (i < boxcar_half_width) {
            for (int j = 0; j < i + boxcar_half_width + 1; j++)
                sum += input_buffer[j];
            average = sum / (double) (boxcar_half_width + i + 1);
        }
        else if (i >= num_pixels - boxcar_half_width) {
            for (int j = i - boxcar_half_width;
                    j < num_pixels;
                    j++)
                sum += input_buffer[j];
            average = sum / (double) (num_pixels - i + boxcar_half_width);
        }
        else {
            for (int j = i - boxcar_half_width;
                    j < i + boxcar_half_width + 1;
                    j++)
                sum += input_buffer[j];
            average = sum / (double) boxcar_width;
        }
        output_buffer[i] = average;
    }
}

void drvUSBQEPro::write_data_files() {
    std::string file_name;
    std::string file_path;
    std::string dir_path;
    int file_write;
    int x_axis_mode;
    int file_index;

    double *x_axis_buffer;

    getIntegerParam(P_fileIndex, &file_index);
    getIntegerParam(P_fileWrite, &file_write);
    getIntegerParam(P_xAxisMode, &x_axis_mode);

    if (x_axis_mode == QEPRO_XAXIS_RAMAN_SHIFT)
        x_axis_buffer = raman_shift_buffer;
    else
        x_axis_buffer = wavelength_buffer;


    if (file_write) {
        // Check that the directory path exists. If not, 
        // put that PV into error.
        char dir_path[2 * BUF_SIZE];
        getStringParam(P_filePath, 2 * BUF_SIZE, dir_path);
        if (!file_exists(dir_path)) {
            setParamAlarmStatus(P_filePath, epicsAlarmState);
            setParamAlarmSeverity(P_filePath, MAJOR_ALARM);
            callParamCallbacks();
        }
        else {
            setParamAlarmStatus(P_filePath, NO_ALARM);
            setParamAlarmSeverity(P_filePath, NO_ALARM);
            callParamCallbacks();
        }

        // Find the next available index
        file_path = create_file_path(
                FILE_DATA,
                file_index);

        while (file_exists(file_path.c_str())) {
            file_index++;
            file_path = create_file_path(
                    FILE_DATA,
                    file_index);
        }

        setIntegerParam(P_fileIndex, file_index);
        callParamCallbacks();

        write_file(
                x_axis_buffer, 
                raw_spectrum_buffer, 
                FILE_RAW_DATA, 
                file_index);
        write_file(
                x_axis_buffer, 
                bg_buffer, 
                FILE_BACKGROUND, 
                file_index);
        write_file(
                x_axis_buffer, 
                raw_bg_buffer, 
                FILE_RAW_BACKGROUND, 
                file_index);
        write_file(
                x_axis_buffer, 
                dark_buffer, 
                FILE_DARK, 
                file_index);
        write_file(
                x_axis_buffer, 
                spectrum_buffer, 
                FILE_DATA, 
                file_index);
        // Increment the file index
        file_index++;
        setIntegerParam(P_fileIndex, file_index);

        callParamCallbacks();
    }
}

void drvUSBQEPro::write_file(
        const double *x_axis_buffer,
        const double *data_buffer,
        const int file_type,
        const int file_index) {

    std::string full_file_name;
    std::string full_file_path;

    std::ofstream outfile;

    full_file_name = create_file_name(
            file_type, 
            file_index);

    full_file_path = create_file_path(
            file_type,
            file_index);

    outfile.open(full_file_path.c_str(), std::ofstream::out);

    write_header(outfile, full_file_name, file_type);

    outfile.precision(4);

    for (int i = 0; i < num_pixels; i++) {
        outfile << std::left << std::setw(12) << std::fixed;
        outfile << x_axis_buffer[i];
        outfile << std::left << std::setw(12) << std::fixed;
        outfile << data_buffer[i];
        outfile << std::endl;
    }

    outfile.close();
}

std::string drvUSBQEPro::create_file_name(
        const int file_type,
        const int file_index) {

    const char* functionName = "create_file_name";

    char file_name[FILE_NAME_SIZE];
    char full_file_name[BUF_SIZE];

    getStringParam(P_fileName, FILE_NAME_SIZE, file_name);

    asynPrint(
            pasynUserSelf,
            ASYN_TRACE_FLOW,
            "%s: file name = %s\n",
            functionName,
            file_name);

    snprintf(full_file_name,
            BUF_SIZE,
            "%s%s_%05d.txt",
            file_name,
            file_extensions[file_type].c_str(),
            file_index);

    // Set the filename data
    setStringParam(P_fullFileName, full_file_name);

    return std::string(full_file_name);
}

std::string drvUSBQEPro::create_file_path(
        const int file_type,
        const int file_index) {

    std::string full_file_name = 
        create_file_name(file_type, file_index);
    std::string full_file_path;

    const char* functionName = "create_file_path";

    char file_path[2 * BUF_SIZE];

    getStringParam(P_filePath, 2 * BUF_SIZE, file_path);

    asynPrint(
            pasynUserSelf,
            ASYN_TRACE_FLOW,
            "%s: file path = %s\n",
            functionName,
            file_path);

    full_file_path = std::string(file_path);
    full_file_path += "/";
    full_file_path += full_file_name;

    asynPrint(
            pasynUserSelf,
            ASYN_TRACE_FLOW,
            "%s: full file path = %s\n",
            functionName,
            full_file_path.c_str());

    // Set the filename data
    setStringParam(P_filePath, file_path);
    setStringParam(P_fullFilePath, full_file_path.c_str());

    return std::string(full_file_path);
}

void drvUSBQEPro::write_header(
        std::ofstream &outfile, 
        std::string full_file_name,
        const int file_type) {
    int integration_time;
    char serial_number[BUF_SIZE];
    char text_buffer[BUF_SIZE];
    int trigger_mode;
    int dark_subtraction;
    int bg_subtraction;
    int nonlinearity_correction;
    int boxcar_width;
    int x_axis_mode;
    int scans_to_average = 1;

    outfile << "QEPro datafile: " << full_file_name << std::endl;
    outfile << "Data type: " << file_descriptions[file_type] << std::endl;

    time_t rawtime;
    struct tm * timeinfo;
    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(text_buffer,
            BUF_SIZE,
            "%FT%T%Z",
            timeinfo);

    outfile << "Date: " << text_buffer << std::endl;
    outfile << "User: " << std::endl;
    getStringParam(P_serialNumber, BUF_SIZE, serial_number);
    outfile << "Spectrometer: " << serial_number << std::endl;

    getIntegerParam(P_triggerMode, &trigger_mode);
    outfile << "Trigger mode: " << trigger_mode << std::endl;

    outfile.precision(5);
    getIntegerParam(P_integrationTime, &integration_time);
    outfile << "Integration time (s): " << std::scientific;
    outfile <<  (double)integration_time/1000000 << std::endl;

    getIntegerParam(P_averages, &scans_to_average);
    outfile << "Scans to average: " << scans_to_average << std::endl;

    getIntegerParam(P_darkSubtract, &dark_subtraction);
    outfile << "Dark correction enabled: ";
    outfile << ((dark_subtraction==0)?"false":"true");
    outfile << std::endl;

    getIntegerParam(P_bgSubtract, &bg_subtraction);
    outfile << "Background subtraction enabled: ";
    outfile << ((bg_subtraction==0)?"false":"true");
    outfile << std::endl;

    getIntegerParam(P_nonLinearity, &nonlinearity_correction);
    outfile << "Nonlinearity correction enabled: ";
    outfile << ((nonlinearity_correction==0)?"false":"true");
    outfile << std::endl;

    getIntegerParam(P_boxcarWidth, &boxcar_width);
    outfile << "Boxcar width: " << boxcar_width << std::endl;

    getIntegerParam(P_xAxisMode, &x_axis_mode);
    outfile << "XAxis mode: ";
    outfile << ((x_axis_mode==0)?"Wavelength (nm)":"Raman Shifts");
    outfile << std::endl;

    outfile << "Number of pixels in spectrum: " << num_pixels << std::endl;

    outfile << ">>>>> Begin Spectral Data <<<<<" << std::endl;
}

bool drvUSBQEPro::file_exists(const char *file_name) {
    struct stat buffer;   
    return (stat(file_name, &buffer) == 0); 
}

extern "C" {

    /** EPICS iocsh callable function to call constructor for the drvUSBQEPro class.
     *   * \param[in] portName The name of the asyn port driver to be created.
     *     * \param[in] maxPoints The maximum  number of points in the volt and time arrays */
    int drvUSBQEProConfigure(const char *portName, int maxPoints, double laser) {
        new drvUSBQEPro(portName, maxPoints, laser);
        return(asynSuccess);
    }


    /* EPICS iocsh shell commands */

    static const iocshArg initArg0 = { "portName", iocshArgString};
    static const iocshArg initArg1 = { "max points",iocshArgInt};
    static const iocshArg initArg2 = { "laser",iocshArgDouble};
    static const iocshArg * const initArgs[] = {
        &initArg0,
        &initArg1,
        &initArg2
    };

    static const iocshFuncDef initFuncDef = {"drvUSBQEProConfigure",3,initArgs};

    static void initCallFunc(const iocshArgBuf *args) {
        drvUSBQEProConfigure(args[0].sval, args[1].ival, args[2].dval);
    }

    void drvUSBQEProRegister(void) {
        iocshRegister(&initFuncDef, initCallFunc);
    }

    epicsExportRegistrar(drvUSBQEProRegister);
}

static void worker(void *pPvt) {
    drvUSBQEPro *ptr = (drvUSBQEPro *)pPvt;
    ptr->getSpectrumThread(ptr);
}
