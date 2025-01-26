#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "rtp_h26x.h"

static int g_sockfd;
static int g_cliaddr;

void rtp_h26x_init(int port)
{
    int sockfd;
    struct sockaddr_in servaddr;
    socklen_t len;

    // Create UDP socket
    if ((g_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

	fprintf(stdout, "Socket created\n");

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&g_cliaddr, 0, sizeof(g_cliaddr));

    // Fill server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    // Bind the socket with the server address
    if (bind(g_sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

	fprintf(stdout, "Socket bind on port %i\n", port);
}

void rtp_h26x_deinit(void)
{
    close(g_sockfd);
}



int rtp_pkt_rcv(uint8_t *buffer)
{
    int bytes_rcvd = 0;
    fd_set readfds;
    struct timeval tv;
    int retval;

    FD_ZERO(&readfds);
    FD_SET(g_sockfd, &readfds);

    // Set timeout to 1 second
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    retval = select(g_sockfd + 1, &readfds, NULL, NULL, &tv);

    if (retval == -1) {
        perror("select failed");
    } else if (retval != 0) {
        if (FD_ISSET(g_sockfd, &readfds)) {
            int len = sizeof(g_cliaddr);
            bytes_rcvd = recvfrom(g_sockfd, buffer, MAX_RTP_PACKET_SIZE, MSG_WAITALL, (struct sockaddr *)&g_cliaddr, &len);
            if (bytes_rcvd < 0) {
                perror("recvfrom failed");
            }
        }
    }
    return bytes_rcvd;
}

static bool frag_nalu_started;

void _h26x_payload_reset(int *nalu_size)
{
    frag_nalu_started = false;
    *nalu_size = 0;
}



bool _h265_payload_read_buffer(uint8_t *payload, int payload_size, uint8_t *nalu, int *nalu_size)
{
    bool end_nalu = false;
    // get nalu type
    u_int8_t nalu_type = (payload[0] >> 1) & 0x3F;
    if (nalu_type == 49)
    {
        // Fragmentation unit (fragmented NAL over multiple packets)
        /*


            +---------------+---------------+
            |0|1|2|3|4|5|6|7|0|1|2|3|4|5|6|7|
            +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
            |F|   Type    |  LayerId  | TID |
            +-------------+-----------------+
            
   Figure 1: The Structure of the HEVC NAL Unit Header

        0                   1                   2                   3
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |    PayloadHdr (Type=49)       |   FU header   | DONL (cond)   |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-|
   | DONL (cond)   |                                               |
   |-+-+-+-+-+-+-+-+                                               |
   |                         FU payload                            |
   |                                                               |
   |                               +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                               :...OPTIONAL RTP padding        |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

                Figure 9: The Structure of an FU

   The fields in the payload header are set as follows.  The Type field
   MUST be equal to 49.  The fields F, LayerId, and TID MUST be equal to
   the fields F, LayerId, and TID, respectively, of the fragmented NAL
   unit.

   The FU header consists of an S bit, an E bit, and a 6-bit FuType
   field, as shown in Figure 10.

   +---------------+
   |0|1|2|3|4|5|6|7|
   +-+-+-+-+-+-+-+-+
   |S|E|  FuType   |
   +---------------+
        */

        u_int8_t fu_header = payload[2];
        bool fu_start = fu_header & 0x80;
        bool fu_end = fu_header & 0x40;
        u_int8_t fu_type = fu_header & 0x3F;
        if (fu_start)
        {
            // start of fragment
            frag_nalu_started = true;
            nalu[0] = 0;
            nalu[1] = 0;
            nalu[2] = 0;
            nalu[3] = 1;
            nalu[4] = (payload[0] & 0x81) | (fu_type << 1);     
            nalu[5] = payload[1];
            *nalu_size = 6;
        }

        // copy data
        if (frag_nalu_started)
        {
            memcpy(nalu + *nalu_size, payload + 3, payload_size - 3);
            *nalu_size += payload_size - 3;
        }

        // check if this is the end of the fragment
        if (fu_end)
        {
            end_nalu = true;
            frag_nalu_started = false;

            /*
            if (fu_type == 19)
            {
                fprintf(stdout, "Iframe fragmented NAL unit type %i, size %i\n", fu_type, *nalu_size);
            }
            else
            {
                fprintf(stdout, "Non-Iframe fragmented NAL unit type %i, size %i\n", fu_type, *nalu_size);
            } 
            */           
        }        
    }
    else
    {
        // Single NAL unit
        nalu[0] = 0;
        nalu[1] = 0;
        nalu[2] = 0;
        nalu[3] = 1;
        memcpy(nalu + 4, payload, payload_size);
        *nalu_size = payload_size + 4;
        end_nalu = true;
        /*
        if (nalu_type == 32)
        {
            fprintf(stdout, "VPS single NAL unit type %i, size %i\n", nalu_type, *nalu_size);
        }
        else if (nalu_type == 33)
        {
            fprintf(stdout, "SPS single NAL unit type %i, size %i\n", nalu_type, *nalu_size);
        }
        else if (nalu_type == 34)
        {
            fprintf(stdout, "PPS single NAL unit type %i, size %i\n", nalu_type, *nalu_size);
        }
        else
        {
            fprintf(stdout, "single NAL unit type %i, size %i\n", nalu_type, *nalu_size);
        }
        */
    }

    return end_nalu;
}

bool _h264_payload_read_buffer(uint8_t *payload, int payload_size, uint8_t *nalu, int *nalu_size)
{
    bool end_nalu = false;

    // TODO

    return end_nalu;
}


#define RTP_HEADER_SIZE 12

// unpack rtp h26x payload according to RFC6184 (H264 over RTP) and RFC7798 (H265 over RTP)
bool rtp_h26x_read_buffer(uint8_t *buffer, int n, uint8_t *nalu, int *nalu_size, bool h265)
{
    bool end_nalu = false;
    static bool first_rtp_pkt = true;
    static bool frag_nalu_started = false;
    uint8_t* payload;
    int payload_size;

    // reset nalu state and modes if it's the first rtp packet
    if (first_rtp_pkt)
    {
        _h26x_payload_reset(nalu_size);
    }

    if (n < RTP_HEADER_SIZE)
    {
        fprintf(stdout, "RTP packet too small %i\n", n);
    }
    else
    {
        // check if we've missed rtp packets
        static uint16_t last_rtp_seq = 0;
        uint16_t current_rtp_seq = (((uint16_t)buffer[2]) << 8) | buffer[3];
        if (!first_rtp_pkt && ((last_rtp_seq + 1) != current_rtp_seq))
        {
            fprintf(stdout, "Missed RTP packets from %i to %i\n", last_rtp_seq, current_rtp_seq);
            // reset nalu
            _h26x_payload_reset(nalu_size);
        }
        else
        {
            // it seems to be a valid rtp packet, unpack it
            payload = buffer + RTP_HEADER_SIZE;
            payload_size = n - RTP_HEADER_SIZE;

            // TODO: extract Timestamp from RTP header at 90Khz

            if (h265)
            {
                end_nalu = _h265_payload_read_buffer(payload, payload_size, nalu, nalu_size);
            }
            else
            {
                end_nalu = _h264_payload_read_buffer(payload, payload_size, nalu, nalu_size);
            }
        }
        first_rtp_pkt = false;
        last_rtp_seq = current_rtp_seq;
    }
    return end_nalu;
}