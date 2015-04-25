/**
* This file is autogenerated by nRFgo Studio 1.18.0.9
*/

#ifndef SETUP_MESSAGES_H__
#define SETUP_MESSAGES_H__

#include "hal_platform.h"
#include "aci.h"


#define SETUP_ID 3
#define SETUP_FORMAT 3 /** nRF8001 D */
#define ACI_DYNAMIC_DATA_SIZE 208

/* Service: Gap - Characteristic: Device name - Pipe: SET */
#define PIPE_GAP_DEVICE_NAME_SET          1
#define PIPE_GAP_DEVICE_NAME_SET_MAX_SIZE 10

/* Service: GATT - Characteristic: Service Changed - Pipe: TX_ACK */
#define PIPE_GATT_SERVICE_CHANGED_TX_ACK          2
#define PIPE_GATT_SERVICE_CHANGED_TX_ACK_MAX_SIZE 4

/* Service: Device Information - Characteristic: Software Revision String - Pipe: SET */
#define PIPE_DEVICE_INFORMATION_SOFTWARE_REVISION_STRING_SET          3
#define PIPE_DEVICE_INFORMATION_SOFTWARE_REVISION_STRING_SET_MAX_SIZE 20

/* Service: NMX - Characteristic: NMX TX - Pipe: TX */
#define PIPE_NMX_NMX_TX_TX          4
#define PIPE_NMX_NMX_TX_TX_MAX_SIZE 20

/* Service: NMX - Characteristic: NMX TX - Pipe: SET */
#define PIPE_NMX_NMX_TX_SET          5
#define PIPE_NMX_NMX_TX_SET_MAX_SIZE 20

/* Service: NMX - Characteristic: NMX RX - Pipe: RX */
#define PIPE_NMX_NMX_RX_RX          6
#define PIPE_NMX_NMX_RX_RX_MAX_SIZE 20

/* Service: NMX - Characteristic: NMX RX - Pipe: RX_ACK_AUTO */
#define PIPE_NMX_NMX_RX_RX_ACK_AUTO          7
#define PIPE_NMX_NMX_RX_RX_ACK_AUTO_MAX_SIZE 20


#define NUMBER_OF_PIPES 7

#define SERVICES_PIPE_TYPE_MAPPING_CONTENT {\
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_TX_ACK},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_SET},   \
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_RX_ACK_AUTO},   \
}

#define GAP_PPCP_MAX_CONN_INT 0x50 /**< Maximum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific value requested */
#define GAP_PPCP_MIN_CONN_INT  0x6 /**< Minimum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific value requested */
#define GAP_PPCP_SLAVE_LATENCY 0
#define GAP_PPCP_CONN_TIMEOUT 0xc8 /** Connection Supervision timeout multiplier as a multiple of 10msec, 0xFFFF means no specific value requested */

#define NB_SETUP_MESSAGES 27
#define SETUP_MESSAGES_CONTENT {\
    {0x00,\
        {\
            0x07,0x06,0x00,0x00,0x03,0x02,0x42,0x07,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x07,0x01,0x01,0x00,0x00,0x06,0x00,0x06,\
            0x90,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x1c,0x67,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x10,0x03,0x90,0x03,0xff,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x38,0xff,0xff,0x01,0x2c,0x0a,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x14,0x00,0x00,\
            0x00,0x10,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x05,0x06,0x10,0x54,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x00,0x04,0x04,0x02,0x02,0x00,0x01,0x28,0x00,0x01,0x00,0x18,0x04,0x04,0x05,0x05,0x00,\
            0x02,0x28,0x03,0x01,0x0e,0x03,0x00,0x00,0x2a,0x04,0x14,0x0a,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x1c,0x0a,0x00,0x03,0x2a,0x00,0x01,0x53,0x6c,0x69,0x64,0x65,0x72,0x43,0x74,0x72,0x6c,\
            0x04,0x04,0x05,0x05,0x00,0x04,0x28,0x03,0x01,0x02,0x05,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x38,0x01,0x2a,0x06,0x04,0x03,0x02,0x00,0x05,0x2a,0x01,0x01,0x00,0x00,0x04,0x04,0x05,\
            0x05,0x00,0x06,0x28,0x03,0x01,0x02,0x07,0x00,0x04,0x2a,0x06,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x54,0x04,0x09,0x08,0x00,0x07,0x2a,0x04,0x01,0x06,0x00,0x50,0x00,0x00,0x00,0xc8,0x00,\
            0x04,0x04,0x02,0x02,0x00,0x08,0x28,0x00,0x01,0x01,0x18,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x70,0x04,0x05,0x05,0x00,0x09,0x28,0x03,0x01,0x22,0x0a,0x00,0x05,0x2a,0x26,0x04,0x05,\
            0x04,0x00,0x0a,0x2a,0x05,0x01,0x00,0x00,0x00,0x00,0x46,0x14,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x8c,0x03,0x02,0x00,0x0b,0x29,0x02,0x01,0x00,0x00,0x04,0x04,0x02,0x02,0x00,0x0c,0x28,\
            0x00,0x01,0x0a,0x18,0x04,0x04,0x05,0x05,0x00,0x0d,0x28,0x03,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xa8,0x01,0x02,0x0e,0x00,0x29,0x2a,0x06,0x04,0x15,0x14,0x00,0x0e,0x2a,0x29,0x01,0x6d,\
            0x69,0x6c,0x69,0x6e,0x64,0x75,0x72,0x2e,0x64,0x65,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xc4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x04,0x08,0x07,0x00,0x0f,0x29,0x04,0x01,\
            0x19,0x00,0x00,0x00,0x01,0x00,0x00,0x04,0x04,0x05,0x05,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xe0,0x10,0x28,0x03,0x01,0x02,0x11,0x00,0x28,0x2a,0x04,0x04,0x14,0x06,0x00,0x11,0x2a,\
            0x28,0x01,0x76,0x30,0x2e,0x31,0x2e,0x31,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x04,0x08,0x07,0x00,0x12,\
            0x29,0x04,0x01,0x19,0x00,0x00,0x00,0x01,0x00,0x00,0x04,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x18,0x10,0x10,0x00,0x13,0x28,0x00,0x01,0x50,0xb5,0x0a,0xe8,0x6a,0x20,0x31,0x92,0xba,\
            0x41,0xad,0x62,0x67,0x60,0xe0,0xb8,0x04,0x04,0x13,0x13,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x34,0x14,0x28,0x03,0x01,0x12,0x15,0x00,0xee,0xfc,0xd5,0x4f,0x69,0xcc,0xcc,0x8e,0x67,\
            0x47,0xe8,0xae,0x7b,0x17,0x97,0xf8,0x14,0x04,0x14,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x50,0x15,0x17,0x7b,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x46,0x14,0x03,0x02,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x6c,0x00,0x16,0x29,0x02,0x01,0x00,0x00,0x04,0x04,0x13,0x13,0x00,0x17,0x28,0x03,0x01,\
            0x0c,0x18,0x00,0x4b,0x1b,0x5f,0x06,0xd6,0xe5,0xa0,0xbb,0xc8,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x88,0x4b,0x2a,0xde,0x0a,0xe4,0x45,0xbf,0x44,0x10,0x14,0x00,0x00,0x18,0xe4,0x0a,0x04,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x0c,0x06,0x21,0xa4,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x00,0x2a,0x00,0x01,0x00,0x80,0x04,0x00,0x03,0x00,0x00,0x2a,0x05,0x01,0x00,0x04,0x04,\
            0x00,0x0a,0x00,0x0b,0x2a,0x28,0x01,0x00,0x80,0x04,0x00,0x11,\
        },\
    },\
    {0x00,\
        {\
            0x19,0x06,0x40,0x1c,0x00,0x00,0x17,0x7b,0x03,0x00,0x82,0x04,0x00,0x15,0x00,0x16,0xe4,0x0a,0x04,0x04,\
            0x08,0x04,0x00,0x18,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x50,0x00,0x50,0xb5,0x0a,0xe8,0x6a,0x20,0x31,0x92,0xba,0x41,0xad,0x62,0x00,0x00,0xe0,0xb8,\
            0xee,0xfc,0xd5,0x4f,0x69,0xcc,0xcc,0x8e,0x67,0x47,0xe8,0xae,\
        },\
    },\
    {0x00,\
        {\
            0x17,0x06,0x50,0x1c,0x00,0x00,0x97,0xf8,0x4b,0x1b,0x5f,0x06,0xd6,0xe5,0xa0,0xbb,0xc8,0x4b,0x2a,0xde,\
            0x00,0x00,0x45,0xbf,\
        },\
    },\
    {0x00,\
        {\
            0x12,0x06,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x06,0x06,0xf0,0x00,0x03,0x0a,0x00,\
        },\
    },\
}

#endif
