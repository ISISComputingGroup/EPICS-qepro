#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "drvUSBQEPro.h"
#include "drvUSBQEProOBP.h"

#include "api/SeaBreezeWrapper.h"

int drvUSBQEPro::abort() {

    OBPExchange xfer;

    memset(&xfer, 0, sizeof(xfer));
    xfer.message_type = OBP_MESSAGE_ABORT_ACQUISITION;

    if (!sendOBPMessage(&xfer)) {
        return 0;
    }
    else {
        return -1;
    }

}

int drvUSBQEPro::clear_buffers() {
    OBPExchange xfer;

    memset(&xfer, 0, sizeof(xfer));
    xfer.message_type = OBP_MESSAGE_CLEAR_BUFFERS;

    if (!sendOBPMessage(&xfer)) {
        return 0;
    }
    else {
        return -1;
    }
}


int drvUSBQEPro::start_acquisition() {

    OBPExchange xfer;

    memset(&xfer, 0, sizeof(xfer));
    xfer.message_type = OBP_MESSAGE_START_BUFFERING;

    if (!sendOBPMessage(&xfer)) {
        return 0;
    }
    else {
        return -1;
    }
}

// execute an OBP request-response pair over serial (returns 0 on success)
int drvUSBQEPro::sendOBPMessage(OBPExchange *xfer)
{
    if (!xfer)
    {
        return 1;
    }

    xfer->actual_response_len = 0;

    if (xfer->request_len > 16)
    {
        return 1;
    }

    // check that compiler / architecture hasn't broken anything
    if (sizeof(OBPHeader) != 44)
    {
        return 1;
    }

    ////////////////////////////////////////////////////////////////////////////
    // define outbound message (header, checksum, footer)
    ////////////////////////////////////////////////////////////////////////////

    // header
    OBPHeader header;
    memset(&header, 0, sizeof(header));
    header.start_bytes[0] = 0xc1;
    header.start_bytes[1] = 0xc0;
    header.protocol_version = 0x1100;
    header.flags = 0x0000; // no ACK requested
    header.message_type = xfer->message_type;
    header.immediate_data_len = xfer->request_len;
    for (unsigned i = 0; i < xfer->request_len; i++)
        header.immediate_data[i] = xfer->request[i];
    header.bytes_remaining = 16 + 4; // i.e. 0x14, 16-byte checksum + 4-byte footer

    // checksum
    unsigned char checksum[16];
    memset(checksum, 0, sizeof(checksum));

    // footer
    unsigned char footer[] = { 0xc5, 0xc4, 0xc3, 0xc2 };

    ////////////////////////////////////////////////////////////////////////////
    // assemble into one message
    ////////////////////////////////////////////////////////////////////////////

    // TODO: merge message with header, checksum, and footer, to avoid this
    //       extra storage and copy
    unsigned char message[64];
    memset(message, 0, sizeof(message));

    memcpy((void*) (message + 0),       (void*) &header,  sizeof(header));
    memcpy((void*) (message + 44),      (void*) checksum, sizeof(checksum));
    memcpy((void*) (message + 44 + 16), (void*) footer,   sizeof(footer));

    ////////////////////////////////////////////////////////////////////////////
    // send message
    ////////////////////////////////////////////////////////////////////////////

    write_buffer(message, sizeof(message));

    ////////////////////////////////////////////////////////////////////////////
    // read response, if one is expected
    ////////////////////////////////////////////////////////////////////////////

    if (xfer->response_len > 0 && xfer->response)
    {
        unsigned expected_response_size = sizeof(message) + xfer->extra_response_len;
        unsigned char *response = (unsigned char*) malloc(expected_response_size);

        int bytes_read = read_buffer(response, expected_response_size);
        if ((unsigned)bytes_read != expected_response_size)
            printf("%s: only read %d of expected %u byte response\n",
                    __FUNCTION__, bytes_read, expected_response_size);

        OBPHeader *response_header = (OBPHeader*) response;
        if (0 != response_header->err_no)
        {
            printf("%s: QEPro response contained error: %s\n",
                    __FUNCTION__, getOBPError(response_header->err_no));
            return -1;
        }

        unsigned bytes_copied = 0;

        // extract response from returned immediate_data
        if (response_header->immediate_data_len > 0)
        {
            unsigned bytes_to_copy = response_header->immediate_data_len;
            if (bytes_to_copy + bytes_copied > xfer->response_len)
            {
                printf("%s: dropped %d immediate_data bytes (insufficient room in output buffer)\n",
                        __FUNCTION__, xfer->response_len - (bytes_to_copy + bytes_copied));
                bytes_to_copy = xfer->response_len - (bytes_to_copy + bytes_copied);
            }
            if (bytes_to_copy > 0)
            {
                memcpy(xfer->response, response_header->immediate_data, bytes_to_copy);
                bytes_copied += bytes_to_copy;
            }
        }

        // extract any remaining_data
        if ((unsigned)bytes_read > sizeof(message) && response_header->bytes_remaining > sizeof(checksum) + sizeof(footer))
        {
            // extract remaining_data from response
            unsigned bytes_to_copy = bytes_read - sizeof(message);
            if (bytes_to_copy + bytes_copied > xfer->response_len)
            {
                printf("%s: dropped %d remaining_data bytes (not enough room in output buffer)\n",
                        __FUNCTION__, xfer->response_len - (bytes_to_copy + bytes_copied));
                bytes_to_copy = xfer->response_len - bytes_copied;
            }

            if (bytes_to_copy > 0)
                memcpy(xfer->response+ bytes_copied, response + sizeof(header), bytes_to_copy);
            bytes_copied += bytes_to_copy;
        }

        xfer->actual_response_len = bytes_copied;

        // TODO: validate checksum
        // TODO: validate footer

        // cleanup
        free(response);
    }

    return 0;
}

// convert an OBP header.err_no to human-readable string
const char* drvUSBQEPro::getOBPError(unsigned err_no)
{
    switch (err_no)
    {
        case   0: return "Success";
        case   1: return "Invalid/unsupported protocol";
        case   2: return "Unknown message type";
        case   3: return "Bad checksum";
        case   4: return "Message too large";
        case   5: return "Payload length does not match message type";
        case   6: return "Payload data invalid";
        case   7: return "Device not ready for given message";
        case   8: return "Unknown checksum type";
        case   9: return "Device reset unexpectedly";
        case  10: return "Too many buses";
        case  11: return "Out of memory";
        case  12: return "Desired information does not exist";
        case  13: return "Internal device error";
        case 100: return "Could not decrypt properly";
        case 101: return "Firmware layout invalid";
        case 102: return "Data packet wrong size";
        case 103: return "Hardware revision not compatible with firmware";
        case 104: return "Existing flash map not compatible with firmware";
        case 255: return "Operation/Response deferred.  Operation will take some time to complete.";
    }

    printf("%s: unknown OPB error code (%d)\n", __FUNCTION__, err_no);
    return "UNKNOWN OBP ERROR CODE";
}

// send a buffer 
void drvUSBQEPro::write_buffer(unsigned char *request, size_t len)
{
    int error;
    api->rawUSBBusAccessWrite(
            device_id, 
            usb_feature_id, 
            &error, 
            request, 
            len, 
            STS_REQUEST_ENDPOINT);
}

// read a buffer 
int drvUSBQEPro::read_buffer(unsigned char *response, size_t len)
{
    int total_bytes_read = 0;
    int error;

    total_bytes_read = api->rawUSBBusAccessRead(
            device_id, 
            usb_feature_id, 
            &error, 
            response, 
            len, 
            STS_RESPONSE_ENDPOINT);

    return total_bytes_read;
}

void drvUSBQEPro::read_temperatures() {
    OBPExchange xfer;
    unsigned char sensor_count = 0;

    memset(&xfer, 0, sizeof(xfer));
    xfer.message_type = OBP_MESSAGE_GET_TEMP_SENSOR_COUNT;
    xfer.response_len = sizeof(sensor_count);
    xfer.response = &sensor_count;

    if (!sendOBPMessage(&xfer))
    {
        if (xfer.response_len == xfer.actual_response_len)
        {
            unsigned int temp_sensor_len = sensor_count*sizeof(float);
            float *temp_sensors = (float*)malloc(temp_sensor_len);
            memset(temp_sensors, 0, temp_sensor_len);

            memset(&xfer, 0, sizeof(xfer));
            xfer.message_type = OBP_MESSAGE_READ_ALL_TEMP_SENSORS;
            xfer.response_len = temp_sensor_len;
            xfer.response = (unsigned char *)temp_sensors;

            if (!sendOBPMessage(&xfer)) {
                for (unsigned char i = 0; i < sensor_count; i++) {
                    temperatures[i] = temp_sensors[i];
                }
            }
            free(temp_sensors);
            temp_sensors = NULL;
        }
        else
            asynPrint(
                    pasynUserSelf,
                    ASYN_TRACE_ERROR,
                    "read_temperatures: expected %u bytes back from 0x%08x, received %u\n",
                    xfer.response_len, 
                    xfer.message_type, 
                    xfer.actual_response_len);
    }
    else {
        asynPrint(
                pasynUserSelf,
                ASYN_TRACE_ERROR,
                "read_temperatures: unable to execute GET_TEMP_SENSOR_COUNT transaction\n");
    }
}
