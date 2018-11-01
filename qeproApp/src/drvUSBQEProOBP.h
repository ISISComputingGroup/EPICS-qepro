#ifndef DRVUSBQEPROOBP_H
#define DRVUSBQEPROOBP_H

// File: drvUSBQEProOBP.h
// Author: Wayne Lewis
// Date: 2018-05-25
//
// Description: 
// Header file for commands to QEPro using Ocean Optics Binary Protocol (OBP)
//
// Acknowledgements:
// Significant portions of this has been copied from OBP Seabreeze library.
//
// Licence:
// OBP licence:
//
/*
* LICENSE:
*
* SeaBreeze Copyright (C) 2015, Ocean Optics Inc
*
* Permission is hereby granted, free of charge, to any person obtaining
* a copy of this software and associated documentation files (the
* "Software"), to deal in the Software without restriction, including
* without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to
* permit persons to whom the Software is furnished to do so, subject
* to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

////////////////////////////////////////////////////////////////////////////////
// Preprocessor Macros
////////////////////////////////////////////////////////////////////////////////

// just a few of the many commands available within OBP
#define OBP_MESSAGE_RESET_DEFAULTS          0x00000001L
#define OBP_MESSAGE_GET_BAUD_RATE           0x00000800L
#define OBP_MESSAGE_SET_BAUD_RATE           0x00000810L
#define OBP_MESSAGE_GET_FLOW_CONTROL        0x00000804L
#define OBP_MESSAGE_SET_FLOW_CONTROL        0x00000814L
#define OBP_MESSAGE_GET_HARDWARE_VERSION    0x00000080L
#define OBP_MESSAGE_GET_FIRMWARE_VERSION    0x00000090L
#define OBP_MESSAGE_GET_SERIAL_NUMBER       0x00000100L
#define OBP_MESSAGE_GET_SERIAL_NUMBER_LEN   0x00000101L
#define OBP_MESSAGE_ABORT_ACQUISITION       0x00100000L

// temperature sensor commands
#define OBP_MESSAGE_GET_TEMP_SENSOR_COUNT   0x00400000L
#define OBP_MESSAGE_READ_ALL_TEMP_SENSORS   0x00400002L

// TEC commands
#define OBP_MESSAGE_GET_TEC_ENABLED         0x00420000L
#define OBP_MESSAGE_GET_TEC_STABLE          0x00420003L
#define OBP_MESSAGE_GET_TEC_SETPOINT        0x00420001L
#define OBP_MESSAGE_GET_TEC_TEMPERATURE     0x00420004L

// Integration time
#define OBP_MESSAGE_GET_INTEGRATION_TIME    0x00110000L
#define OBP_MESSAGE_SET_INTEGRATION_TIME    0x00110010L

// Buffer Specific Commands
#define OBP_MESSAGE_GET_DEVICE_IS_IDLE      0x00100908L
#define OBP_MESSAGE_GET_MAX_BUFFER_SIZE     0x00100820L
#define OBP_MESSAGE_GET_BUFFER_SIZE         0x00100822L
#define OBP_MESSAGE_GET_NUM_IN_BUFFER       0x00100900L
#define OBP_MESSAGE_SET_BUFFER_SIZE         0x00100832L
#define OBP_MESSAGE_GET_BUFFERED_SPECTRA    0x00100928L
#define OBP_MESSAGE_CLEAR_BUFFERS           0x00100830L
#define OBP_MESSAGE_START_BUFFERING         0x00100902L  // Acquire into Buffer (start buffer spectra in accordance with trigger mode)

// TODO: replace with buffered commands
#define OBP_MESSAGE_GET_CORRECTED_SPECTRUM  0x00101000L
#define OBP_MESSAGE_SET_SUBSPECTRUM_SPEC    0x00102010L
#define OBP_MESSAGE_GET_SUBSPECTRUM         0x00102080L

// wavelength coefficients
#define OBP_MESSAGE_GET_WL_COEFF_COUNT      0x00180100L
#define OBP_MESSAGE_GET_WL_COEFF            0x00180101L
#define OBP_MESSAGE_SET_WL_COEFF            0x00180111L

// non-linearity corefficients
#define OBP_MESSAGE_GET_NLC_COEFF_COUNT     0x00181100L
#define OBP_MESSAGE_GET_NLC_COEFF           0x00181101L
#define OBP_MESSAGE_SET_NLC_COEFF           0x00181101L

// convert 2-byte LSB-MSB to native
#define LITTLE_ENDIAN_SHORT(base) (((base)[1] << 8) | (base)[0])

// convert 4-byte LSB-MSB to native
#define LITTLE_ENDIAN_WORD(base) (((base)[3] << 24) | ((base)[2] << 16) | ((base)[1] << 8) | (base)[0])


typedef struct OBPExchange_s
{
    unsigned int   message_type;        // (input)  OBP message code
    unsigned int   request_len;         // (input)  how much request parameter data to send as "OBP immediate"
    unsigned char *request;             // (input)  pointer to request parameter data
    unsigned int   response_len;        // (input)  max size of response data (output_data allocated size)
    unsigned char *response;            // (output) pointer to where response data should be written
    unsigned int   extra_response_len;  // (input)  how many bytes of non-immediate response expected (beyond 16-byte immediate)
    unsigned int   actual_response_len; // (output) how much data actually came back in response
} OBPExchange;

// Taken directly from page 22 of the QEPro Data Sheet
typedef struct OBPHeader_s
{                                       // byte offset
    unsigned char  start_bytes[2];      // 0-1: 0xC1, 0xC0
    unsigned short protocol_version;    // 2-3: 0x1100
    unsigned short flags;               // 4-5
    unsigned short err_no;              // 6-7
    unsigned int   message_type;        // 8-11
    unsigned int   regarding;           // 12-15
    unsigned char  reserved[6];         // 16-21
    unsigned char  checksum_type;       // 22
    unsigned char  immediate_data_len;  // 23
    unsigned char  immediate_data[16];  // 24-39
    unsigned int   bytes_remaining;     // 40-43
} OBPHeader;                            // 44 bytes total

typedef struct OBP_Metadata_s
{                                       // byte offset
    //-- metadata --
    unsigned int       spec_count;          // 0-3
    unsigned long long tick_count __attribute__((packed)); // 4-11 (packed to enforce integration_time alignment)
    unsigned int       integration_time ;    // 12-15
    unsigned short     reserved1;           // 16-17
    unsigned char      trigger_mode;        // 18
    unsigned char      reserved2[13];       // 19-31
} OBP_Metadata; // 32 bytes total

#endif
