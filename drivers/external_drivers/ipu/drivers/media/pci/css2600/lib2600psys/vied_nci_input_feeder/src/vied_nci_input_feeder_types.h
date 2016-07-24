#ifndef _INPUT_FEEDER_TYPES_H_
#define _INPUT_FEEDER_TYPES_H_

typedef unsigned int  uint;
typedef unsigned char uchar;

#define INPUT_FEEDER_DEV_ID    0
#define INPUT_FEEDER_MAX_SIDS  5

// Stream IDs
#define BAYER0_ID     0
#define BAYER1_ID     1
#define YUV420_ID     2
#define RGB0_ID       3
#define RGB1_ID       4

// Message IDs
#define INPUT_FEEDER_MSG_SUCCESSFUL                          0
#define INPUT_FEEDER_MSG_STATIC_CONF_NOT_DONE               -2
#define INPUT_FEEDER_MSG_FIFO_SIZE_EXCEEDED                 -3
#define INPUT_FEEDER_MSG_NUM_UNITS_PER_FETCH_EXCEEDED       -4
#define INPUT_FEEDER_MSG_UNITS_PER_BUFFER_EXCEEDED          -5
#define INPUT_FEEDER_MSG_SPAN_WIDTH_EXCEEDED                -6
#define INPUT_FEEDER_MSG_SPAN_HEIGHT_EXCEEDED               -7
#define INPUT_FEEDER_MSG_REGION_STRIDE_EXCEEDED             -8



#endif /* _INPUT_FEEDER_TYPES_H_ */

