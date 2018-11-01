#ifndef DRVUSBQEPRO_H
#define DRVUSBQEPRO_H
#include <iocsh.h>
#include <epicsExport.h>
#include <asynPortDriver.h>

#include <libusb-1.0/libusb.h>

#include <fstream>

#include "drvUSBQEProOBP.h"
#include "api/seabreezeapi/SeaBreezeAPI.h"

#define QEProNumSpecs           "NumSpecs"              /* asynInt32        ro */
#define QEProId                 "NrBoard"               /* asynInt32        ro */
#define QEProName               "Name"                  /* asynOctet        ro */ 
#define QEProFirmwareVersion    "FirmwareVersion"       /* asynOctet        ro */
#define QEProFirmwareModel      "FirmwareModel"         /* asynOctet        ro */    
#define QEProSerialNumber       "SerialNumber"          /* asynOctet        ro */
#define QEProNumberOfPixels     "NumberOfPixels"        /* asynInt32        rw */
#define QEProNumberOfDarkPixels "NumberOfDarkPixels"    /* asynInt32        rw */
#define QEProIntegrationTime    "IntegrationTime"       /* asynInt32        rw */
#define QEProMaxIntegrationTime "MaxIntegrationTime"    /* asynInt32        rw */
#define QEProMinIntegrationTime "MinIntegrationTime"    /* asynInt32        rw */
#define QEProMaxIntensity       "MaxIntensity"          /* asynInt32        rw */
#define QEProBoxcarWidth        "BoxcarWidth"           /* asynInt32        rw */
#define QEProElectricDark       "ElectricDark"          /* asynInt32        rw */
#define QEProDetectorTemperature "DetectorTemperature"  /* asynInt32        rw */
#define QEProBoardTemperature   "BoardTemperature"      /* asynInt32        rw */
#define QEProTempSetPoint       "TempSetPoint"          /* asynInt32        rw */
#define QEProTriggerMode        "TriggerMode"           /* asynInt32        rw */
#define QEProNonLinearity       "NonLinearity"          /* asynInt32        rw */
#define QEProDecouple           "Decouple"              /* asynInt32        rw */
#define QEProLEDIndicator       "LEDIndicator"          /* asynInt32        rw */
#define QEProAverages           "Averages"              /* asynInt32        rw */
#define QEProXAxisNm            "XAxisNm"               /* asynFloat64Array ro */
#define QEProXAxisRs            "XAxisRs"               /* asynFloat64Array ro */
#define QEProSpectrum           "Spectrum"              /* asynFloat64Array ro */
#define QEProLaser              "Laser"                 /* asynFloat64      ro */
#define QEProConnected          "Connected"             /* asynInt32        ro */
#define QEProAcqMode            "AcqMode"               /* asynInt32        rw */
#define QEProAcqStart           "AcqStart"              /* asynInt32        rw */
#define QEProAcqStop            "AcqStop"               /* asynInt32        rw */
#define QEProAcqSts             "AcqSts"                /* asynInt32        ro */
#define QEProFileWrite          "FileWrite"             /* asynInt32        rw */
#define QEProFilePath           "FilePath"              /* asynOctet        rw */
#define QEProFileName           "FileName"              /* asynOctet        rw */
#define QEProFullFileName       "FullFileName"          /* asynOctet        rw */
#define QEProFullFilePath       "FullFilePath"          /* asynOctet        rw */
#define QEProFileIndex          "FileIndex"             /* asynInt32        rw */
#define QEProXAxisMode          "XAxisMode"             /* asynInt32        rw */
#define QEProXAxis              "XAxis"                 /* asynFloat64Array ro */
#define QEProCPUTemperature     "CPUTemperature"        /* asynFloat64      ro */
#define QEProPCBTemperature     "PCBTemperature"        /* asynFloat64      ro */
#define QEProDetTemperature     "DetTemperature"        /* asynFloat64      ro */
#define QEProROI0LowWavelength  "ROI0LowWavelength"     /* asynFloat64      ro */
#define QEProROI0HighWavelength "ROI0HighWavelength"    /* asynFloat64      ro */
#define QEProROI1LowWavelength  "ROI1LowWavelength"     /* asynFloat64      ro */
#define QEProROI1HighWavelength "ROI1HighWavelength"    /* asynFloat64      ro */
#define QEProROI0Sum            "ROI0Sum"               /* asynFloat64      ro */
#define QEProROI1Sum            "ROI1Sum"               /* asynFloat64      ro */
#define QEProROI0Fraction       "ROI0Fraction"          /* asynFloat64      ro */
#define QEProROI1Fraction       "ROI1Fraction"          /* asynFloat64      ro */
#define QEProDarkAcq            "DarkAcq"               /* asynInt32        rw */
#define QEProDarkSubtract       "DarkSubtract"          /* asynInt32        rw */
#define QEProDarkSpectrum       "DarkSpectrum"          /* asynFloat64Array ro */
#define QEProDarkValid          "DarkValid"             /* asynInt32        ro */
#define QEProDarkValidOverride  "DarkValidOverride"     /* asynInt32        rw */
#define QEProBGAcq              "BGAcq"                 /* asynInt32        rw */
#define QEProBGSubtract         "BGSubtract"            /* asynInt32        rw */
#define QEProBGSpectrum         "BGSpectrum"            /* asynFloat64Array ro */
#define QEProBGValid            "BGValid"               /* asynInt32        ro */
#define QEProBGValidOverride    "BGValidOverride"       /* asynInt32        ro */

#define POLL_TIME 0.5

#define QEPRO_ACQ_MODE_OFF          0
#define QEPRO_ACQ_MODE_SINGLE       1
#define QEPRO_ACQ_MODE_CONTINUOUS   2

#define QEPRO_XAXIS_WAVELENGTH      0
#define QEPRO_XAXIS_RAMAN_SHIFT     1

#define FILE_NAME_SIZE              40
#define BUF_SIZE                    80
#define TS_SIZE                     16

#define STS_REQUEST_ENDPOINT        0x01
#define STS_RESPONSE_ENDPOINT       0x81

#define NUM_TEMP_SENSORS            8
#define CPU_TEMPERATURE             0
#define PCB_TEMPERATURE             2
#define TEC_TEMPERATURE             3

#define NUM_ROIS                    2

class drvUSBQEPro : public asynPortDriver {

public:
    drvUSBQEPro(const char *portName, int maxArraySize, double laser);

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);

    /* Thread function to read spectra from device */
    virtual void getSpectrumThread(void *);

protected:
    int         P_numSpecs;
    #define     FIRST_QEPRO_PARAM P_numSpecs
    int         P_nrBoard;
    int         P_name;
    int         P_firmwareVersion;
    int         P_firmwareModel;
    int         P_serialNumber;
    int         P_numberOfPixels;
    int         P_numberOfDarkPixels;
    int         P_integrationTime;
    int         P_maxIntegrationTime;
    int         P_minIntegrationTime;
    int         P_maxIntensity;
    int         P_boxcarWidth;
    int         P_electricDark;
    int         P_detectorTemperature;
    int         P_boardTemperature;
    int         P_tempSetPoint;
    int         P_triggerMode;
    int         P_nonLinearity;
    int         P_decouple;
    int         P_ledIndicator;
    int         P_averages; 
    int         P_xAxisNm;   // x-Axis in nanometers 
    int         P_xAxisRs;   // x-Axis in Raman shift
    int         P_spectrum;  // Raman spectrum (y-Axis)
    int         P_laser;     // laser wavelength needed for Raman shift calculation
    int         P_connected;     
    int         P_acqMode;     
    int         P_acqStart;     
    int         P_acqStop;     
    int         P_acqSts;     
    int         P_fileWrite;
    int         P_filePath;
    int         P_fileName;
    int         P_fullFileName;
    int         P_fullFilePath;
    int         P_fileIndex;
    int         P_xAxisMode;
    int         P_xAxis;
    int         P_cpuTemperature;
    int         P_pcbTemperature;
    int         P_detTemperature;
    int         P_roi0LowWavelength;
    int         P_roi0HighWavelength;
    int         P_roi1LowWavelength;
    int         P_roi1HighWavelength;
    int         P_roi0Sum;
    int         P_roi1Sum;
    int         P_roi0Fraction;
    int         P_roi1Fraction;
    int         P_darkAcq;
    int         P_darkSubtract;
    int         P_darkSpectrum;
    int         P_darkValid;
    int         P_darkValidOverride;
    int         P_bgAcq;
    int         P_bgSubtract;
    int         P_bgSpectrum;
    int         P_bgValid;
    int         P_bgValidOverride;
    #define LAST_QEPRO_PARAM P_bgValidOverride

private:
    SeaBreezeAPI *api;
    static int zeroIndex;

    //libusb_hotplug_callback_handle hp[2];
    libusb_context *context;

    static const int OOI_VENDOR_ID = 0x2457;

    long *device_ids;
    long device_id;
    long serial_number_feature_id;
    long spectrometer_feature_id;
    long usb_feature_id;
    long nonlinearity_feature_id;
    int spec_index;

    double temperatures[NUM_TEMP_SENSORS];

    bool connected;
    bool ioc_starting;
    bool acquiring;

    void test_connection();
    void allocate_spectrum_buffer();
    void deallocate_spectrum_buffer();
    void boxcar(
            double *output_buffer,
            const double *input_buffer,
            int boxcar_width);
    void convert_nm_to_raman_shift(
            double *raman_shift_buffer,
            const double *wavelength_buffer);
    void set_integration_time();
    void integrate_rois();
    void read_device_name();
    void read_serial_number();
    void read_number_of_pixels();
    void get_wavelengths();

    double roi_low[NUM_ROIS];
    double roi_high[NUM_ROIS];
    double roi_sum[NUM_ROIS];
    double roi_fraction[NUM_ROIS];

    // QEPro functions using OBP
    int abort();
    int clear_buffers();
    int start_acquisition();
    void read_temperatures();

    // OBP support functions
    int sendOBPMessage(OBPExchange *xfer);
    const char* getOBPError(unsigned err_no);
    void write_buffer(unsigned char *request, size_t len);
    int read_buffer(unsigned char *response, size_t len);

    // Internally stored values and flags
    unsigned long integration_time;
    int trigger_mode;
    int num_pixels;
    int num_wavelengths;
    bool dark_valid;
    bool dark_valid_override;
    bool dark_subtract;
    bool dark_acquire;
    bool bg_valid;
    bool bg_valid_override;
    bool bg_subtract;
    bool bg_acquire;

    // Data buffers for raw and calculated spectra
    double *raw_spectrum_buffer;
    double *spectrum_buffer;
    double *dark_buffer;
    double *raw_bg_buffer;
    double *bg_buffer;
    // Data buffers for x-axis values
    double *wavelength_buffer;
    double *raman_shift_buffer;

    // Utility functions
    void subtract_spectra(
            double *result, 
            const double *spectra1, 
            const double *spectra2);
    void update_bg();
    void update_data_spectrum();
    void update_axis_arrays();
    void acquire_dark();
    void acquire_bg();

    // File handling functions
    void write_data_files();
    bool file_exists(const char *file_path);
    std::string create_file_name(
            const int file_type, 
            const int file_index);
    std::string create_file_path(
            const int file_type, 
            const int file_index);
    void write_file(
            const double *x_axis_buffer,
            const double *data_buffer,
            const int file_type,
            const int file_index);
    void write_header(
            std::ofstream &outfile,
            std::string full_file_name,
            const int file_type);

    double m_laser;
    double m_poll_time;

    // Data file variables
    enum files_t {
        FILE_DATA,
        FILE_RAW_DATA,
        FILE_BACKGROUND,
        FILE_RAW_BACKGROUND,
        FILE_DARK,
    };

    std::vector<std::string> file_extensions;
    std::vector<std::string> file_descriptions;

};

#endif
