#ifndef RTP_H26X_H
#define RTP_H26X_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>

#define MAX_RTP_PACKET_SIZE 2048

// Function to initialize the RTP H26X module
void rtp_h26x_init(int port);

// Function to deinitialize the RTP H26X module
void rtp_h26x_deinit(void);

// Function to receive an RTP packet
int rtp_pkt_rcv(uint8_t *buffer);

// Function to process an RTP packet
bool rtp_h26x_read_buffer(uint8_t *buffer, int n, uint8_t *nalu, int *nalu_size, bool h265);

#ifdef __cplusplus
}
#endif

#endif // RTP_H26X_H