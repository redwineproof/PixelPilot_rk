#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <stdint.h>

#define K_TS_BUFFER_SIZE 10

extern void record_nalu_rcv_ts(unsigned long long recv_begin_ts, unsigned long frameNb);
extern void record_frame_decoded_ts(unsigned long frameNb);
extern void record_frame_displayed_ts(unsigned long frameNb);
extern void record_vsync_ts(void);

extern int timestamp_init(void);
extern int timestamp_exit(void);

#endif // TIMESTAMP_H