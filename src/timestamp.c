#include "timestamp.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <errno.h>
#include <stdbool.h>

#include "time_util.h"
#include "osd.h"

#define RCV_PORT 12345 // Port for receiving timestamps
#define SEND_PORT 12346   // Port for sending timestamps
#define PERIOD_MS 1000    // Période en millisecondes
#define TIMEOUT_US 100000 // Timeout en microsecondes
#define PACKET_MAGIC 0xA1B2C3D4

static pthread_t ground_thread;
static int rcv_sockfd, send_sockfd;
static struct sockaddr_in client_addr, send_addr;

// Define htonll and ntohll functions
uint64_t htonll(uint64_t value) {
    if (htonl(1) != 1) {
        return ((uint64_t)htonl(value & 0xFFFFFFFF) << 32) | htonl(value >> 32);
    } else {
        return value;
    }
}

uint64_t ntohll(uint64_t value) {
    if (ntohl(1) != 1) {
        return ((uint64_t)ntohl(value & 0xFFFFFFFF) << 32) | ntohl(value >> 32);
    } else {
        return value;
    }
}

typedef struct {
    unsigned long      frameNb;
    unsigned long long vsync_timestamp;
    unsigned long long framestart_timestamp;
    unsigned long long frameend_timestamp;
    unsigned long long ispframedone_timestamp;
    unsigned long long vencdone_timestamp;
    unsigned long long one_way_delay_ns;
} air_timestamp_buffer_t;

typedef struct {
    unsigned long long rcv_begin_timestamp;
    unsigned long long nal_rcvd_timestamp;
    unsigned long long frame_decoded_timestamp;
    unsigned long long frame_displayed_timestamp;
    unsigned long long vsync_timestamp;
    unsigned long frame_size;
} ground_timestamp_buffer_t;

typedef struct {
    unsigned long long        air_time_ns;
    unsigned long long        ground_time_ns;
    bool                      air_received;
    bool                      air_synced;
    air_timestamp_buffer_t    air;
    ground_timestamp_buffer_t ground;
} air_ground_timestamp_buffer_t;

typedef struct {
    air_ground_timestamp_buffer_t buffer[K_TS_BUFFER_SIZE];
    unsigned long frame_counter;
} air_ground_timestamp_buffers_t;

typedef enum {
    PACKET_TYPE_AIR_TIME,
    PACKET_TYPE_AIR_TIMESTAMPS
} packet_type_t;

typedef struct {
    uint32_t magic; // Magic number for validation
    packet_type_t type;
    union {
        uint64_t air_time_ns;
        air_timestamp_buffer_t air_timestamps;
    } data;
} air_packet_t;

static air_ground_timestamp_buffers_t ts_buffers;

void record_frame_rcv_ts(unsigned long long recv_begin_ts, unsigned long frameNb, unsigned long frame_size) {
    unsigned long long ts = get_time_ns();
    ts_buffers.buffer[frameNb % K_TS_BUFFER_SIZE].ground.rcv_begin_timestamp = recv_begin_ts;
    ts_buffers.buffer[frameNb % K_TS_BUFFER_SIZE].ground.nal_rcvd_timestamp = ts;
    ts_buffers.buffer[frameNb % K_TS_BUFFER_SIZE].ground.frame_size = frame_size;
}

void record_frame_decoded_ts(unsigned long frameNb) {
    unsigned long long ts = get_time_ns();
    ts_buffers.buffer[frameNb % K_TS_BUFFER_SIZE].ground.frame_decoded_timestamp = ts;
}

void record_frame_displayed_ts(unsigned long frameNb) {
    unsigned long long ts = get_time_ns();
    ts_buffers.buffer[frameNb % K_TS_BUFFER_SIZE].ground.frame_displayed_timestamp = ts;
    ts_buffers.frame_counter = frameNb;
}

//#define DEBUG

void record_vsync_ts(void) {
    unsigned long long ts = get_time_ns();
    unsigned long frame_counter = ts_buffers.frame_counter;
    air_ground_timestamp_buffer_t *buf = &ts_buffers.buffer[frame_counter % K_TS_BUFFER_SIZE];
    buf->ground.vsync_timestamp = ts;

    long long adjust_air_to_ground = buf->ground_time_ns - buf->air_time_ns - buf->air.one_way_delay_ns;
   


    if (buf->air_received)
    {
        unsigned long long g2g_latency = (buf->ground.vsync_timestamp - buf->air.vsync_timestamp - adjust_air_to_ground) / 1000;
        osd_publish_uint_fact("timestamp.g2g", NULL, 0, g2g_latency);
        unsigned long long sensor_latency = (buf->air.frameend_timestamp - buf->air.vsync_timestamp) / 1000;
        osd_publish_uint_fact("timestamp.sensor", NULL, 0, sensor_latency);
        unsigned long long isp_latency = (buf->air.ispframedone_timestamp - buf->air.frameend_timestamp) / 1000;
        osd_publish_uint_fact("timestamp.isp", NULL, 0, isp_latency);
        unsigned long long vpe_venc_latency = (buf->air.vencdone_timestamp - buf->air.ispframedone_timestamp) / 1000;
        osd_publish_uint_fact("timestamp.vpe_venc", NULL, 0, vpe_venc_latency);
        unsigned long long transmission_latency = (((long long)buf->ground.nal_rcvd_timestamp) - ((long long)buf->air.vencdone_timestamp) - adjust_air_to_ground) / 1000;
        osd_publish_uint_fact("timestamp.transmission", NULL, 0, transmission_latency);
        unsigned long long decoding_latency = (buf->ground.frame_decoded_timestamp - buf->ground.nal_rcvd_timestamp) / 1000;
        osd_publish_uint_fact("timestamp.decoding", NULL, 0, decoding_latency);
        unsigned long long display_latency = (buf->ground.vsync_timestamp - buf->ground.frame_decoded_timestamp) / 1000;
        osd_publish_uint_fact("timestamp.display", NULL, 0, display_latency);
        unsigned long long frame_size = buf->ground.frame_size;
        osd_publish_uint_fact("timestamp.size", NULL, 0, frame_size);

        #ifdef DEBUG
        fprintf(stdout, "Sensor Vsync to Screen Vsync:     %llu us\n", g2g_latency);
        fprintf(stdout, "Nb: %i, S:%llu I:%llu E:%llu T:%llu D:%llu F:%llu V:%llu, Size: %i, Status: %s\n",
                frame_counter,
                sensor_latency,
                isp_latency,
                vpe_venc_latency,
                transmission_latency,
                decoding_latency,
                (buf->ground.frame_displayed_timestamp - buf->ground.frame_decoded_timestamp) / 1000,
                (buf->ground.vsync_timestamp - buf->ground.frame_displayed_timestamp) / 1000,
                frame_size,
                buf->air_synced == true ? "Synced": "Not synced");
        #endif
    }
    else
    {
        unsigned long long decoding_latency = (buf->ground.frame_decoded_timestamp - buf->ground.nal_rcvd_timestamp) / 1000;
        osd_publish_uint_fact("timestamp.decoding", NULL, 0, decoding_latency);
        unsigned long long display_latency = (buf->ground.vsync_timestamp - buf->ground.frame_decoded_timestamp) / 1000;
        osd_publish_uint_fact("timestamp.display", NULL, 0, display_latency);
        unsigned long long frame_size = buf->ground.frame_size;
        osd_publish_uint_fact("timestamp.size", NULL, 0, frame_size);

        #ifdef DEBUG
        fprintf(stdout, "Frame Rcv to Screen Vsync:     %llu us\n",
            (buf->ground.vsync_timestamp - buf->ground.nal_rcvd_timestamp) / 1000);
        fprintf(stdout, "Nb: %i, D:%llu F:%llu V:%llu, Size: %i\n",
                frame_counter,
                (buf->ground.frame_decoded_timestamp - buf->ground.nal_rcvd_timestamp) / 1000,
                (buf->ground.frame_displayed_timestamp - buf->ground.frame_decoded_timestamp) / 1000,
                (buf->ground.vsync_timestamp - buf->ground.frame_displayed_timestamp) / 1000,
                buf->ground.frame_size);
        #endif
    }


    // reset validity
    buf->air_received = false;
    buf->air_synced = false;
}


extern int signal_flag;

void *ground_thread_func(void *arg) {

	struct timespec ts;
	unsigned long long air_time_ns;
	unsigned long long ground_time_ns;
	unsigned long long ground_time_ns_network;
	air_packet_t air_packet;
    packet_type_t type;
    uint32_t magic;
    socklen_t addr_len = sizeof(client_addr);
    ssize_t packet_size;

    while (!signal_flag) {

        packet_size = recvfrom(rcv_sockfd, &air_packet, sizeof(air_packet), 0, (struct sockaddr *)&client_addr, &addr_len);
        if (packet_size < 0) {
            if (errno == EWOULDBLOCK || errno == EAGAIN) {
                //fprintf(stderr, "Receive timeout\n");
            } else {
                perror("Failed to receive packet");
                break;
            }
        }

		else
		{
            // Validate the magic number
            magic = ntohl(air_packet.magic);
            //fprintf(stdout, "Magic: %x\n", magic);
            if (magic != PACKET_MAGIC) {
                fprintf(stderr, "Invalid packet received. Ignoring.\n");
                continue;
            }

            type = ntohl(air_packet.type);
			if (type == PACKET_TYPE_AIR_TIME) {
				air_time_ns = ntohll(air_packet.data.air_time_ns);
				// Capturer le temps "ground"
				clock_gettime(CLOCK_MONOTONIC, &ts);
				ground_time_ns = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
			
				// Envoyer le temps "ground" au système "air"
				ground_time_ns_network = htonll(ground_time_ns);

                if (sendto(send_sockfd, &ground_time_ns_network, sizeof(ground_time_ns_network), 0, (struct sockaddr *)&send_addr, sizeof(send_addr)) < 0) {
                    perror("Failed to send response packet");
                }
			}
			else if (type == PACKET_TYPE_AIR_TIMESTAMPS) {
				air_timestamp_buffer_t *air_timestamps = &air_packet.data.air_timestamps;
				// Convertir les champs en host byte order
				unsigned long frameNb = air_timestamps->frameNb;
				air_timestamps->frameNb = ntohl(air_timestamps->frameNb);
				air_timestamps->vsync_timestamp = ntohll(air_timestamps->vsync_timestamp);
				air_timestamps->framestart_timestamp = ntohll(air_timestamps->framestart_timestamp);
				air_timestamps->frameend_timestamp = ntohll(air_timestamps->frameend_timestamp);
				air_timestamps->ispframedone_timestamp = ntohll(air_timestamps->ispframedone_timestamp);
				air_timestamps->vencdone_timestamp = ntohll(air_timestamps->vencdone_timestamp);
				air_timestamps->one_way_delay_ns = ntohll(air_timestamps->one_way_delay_ns);

				// store it
				memcpy(&ts_buffers.buffer[air_timestamps->frameNb % K_TS_BUFFER_SIZE].air, air_timestamps, sizeof(air_timestamp_buffer_t));
				ts_buffers.buffer[air_timestamps->frameNb % K_TS_BUFFER_SIZE].air_time_ns = air_time_ns;
				ts_buffers.buffer[air_timestamps->frameNb % K_TS_BUFFER_SIZE].ground_time_ns = ground_time_ns;

				// set validity
				ts_buffers.buffer[air_timestamps->frameNb % K_TS_BUFFER_SIZE].air_received = true;
				if (air_timestamps->one_way_delay_ns) {
					ts_buffers.buffer[air_timestamps->frameNb % K_TS_BUFFER_SIZE].air_synced = true;
				}
			}
			else {
				// Reset to "receive air_time" state
				fprintf(stderr, "Invalid packet type received. Resetting to receive air_time state.\n");
				continue;
			}
		}

    }
    return NULL;
}



int timestamp_init(void)
{
    struct sockaddr_in server_addr;

    // Créer un socket UDP pour recevoir
    if ((rcv_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Failed to create receive socket");
        return 1;
    }

    // Configurer l'adresse du serveur pour recevoir
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(RCV_PORT);

    // Lier le socket à l'adresse et au port
    if (bind(rcv_sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Failed to bind receive socket");
        close(rcv_sockfd);
        return 1;
    }

    // Configurer le timeout pour recvfrom
    struct timeval timeout = { .tv_sec = 0, .tv_usec = TIMEOUT_US };
    if (setsockopt(rcv_sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        perror("Failed to set receive socket timeout");
        close(rcv_sockfd);
        return 1;
    }

    // Créer un socket UDP pour envoyer
    if ((send_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Failed to create send socket");
        close(rcv_sockfd);
        return 1;
    }

    // Configurer l'adresse pour envoyer
    memset(&send_addr, 0, sizeof(send_addr));
    send_addr.sin_family = AF_INET;
    send_addr.sin_port = htons(SEND_PORT);
    if (inet_pton(AF_INET, "127.0.0.1", &send_addr.sin_addr) <= 0) {
        perror("Invalid send address");
        close(rcv_sockfd);
        close(send_sockfd);
        return 1;
    }


	// Créer et démarrer le thread pour le système "ground"
	if (pthread_create(&ground_thread, NULL, ground_thread_func, NULL) != 0) {
		perror("Failed to create ground thread");
        close(rcv_sockfd);
        close(send_sockfd);
		return 1;
	}

	return 0;
}

int timestamp_exit(void)
{
	pthread_join(ground_thread, NULL);

    // Fermer le socket à la fin du programme
    close(rcv_sockfd);
    close(send_sockfd);
	return 0;
}