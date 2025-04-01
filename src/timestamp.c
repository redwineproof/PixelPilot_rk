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

#include "time_util.h"

#define GROUND_PORT 12345 // Port du système "ground"
#define PERIOD_MS 1000    // Période en millisecondes
#define TIMEOUT_US 100000 // Timeout en microsecondes

static int sockfd = -1;

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
    air_timestamp_buffer_t    air;
    ground_timestamp_buffer_t ground;
} air_ground_timestamp_buffer_t;

typedef struct {
    air_ground_timestamp_buffer_t buffer[K_TS_BUFFER_SIZE];
    unsigned long frame_counter;
} air_ground_timestamp_buffers_t;


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

#define DEBUG

void record_vsync_ts(void) {
    unsigned long long ts = get_time_ns();
    unsigned long frame_counter = ts_buffers.frame_counter;
    air_ground_timestamp_buffer_t *buf = &ts_buffers.buffer[frame_counter % K_TS_BUFFER_SIZE];
    buf->ground.vsync_timestamp = ts;

    long long adjust_air_to_ground = buf->ground_time_ns - buf->air_time_ns - buf->air.one_way_delay_ns;

#ifdef DEBUG
    fprintf(stdout, "Sensor Vsync to Screen Vsync:     %llu us\n",
            (buf->ground.vsync_timestamp - buf->air.vsync_timestamp - adjust_air_to_ground) / 1000);
    fprintf(stdout, "S:%llu I:%llu E:%llu T:%llu D:%llu F:%llu V:%llu, Size: %i\n",
            (buf->air.frameend_timestamp - buf->air.vsync_timestamp) / 1000,
            (buf->air.ispframedone_timestamp - buf->air.frameend_timestamp) / 1000,
            (buf->air.vencdone_timestamp - buf->air.ispframedone_timestamp) / 1000,
            (((long long)buf->ground.nal_rcvd_timestamp) - ((long long)buf->air.vencdone_timestamp) - adjust_air_to_ground) / 1000,
            (buf->ground.frame_decoded_timestamp - buf->ground.nal_rcvd_timestamp) / 1000,
            (buf->ground.frame_displayed_timestamp - buf->ground.frame_decoded_timestamp) / 1000,
            (buf->ground.vsync_timestamp - buf->ground.frame_displayed_timestamp) / 1000,
            buf->ground.frame_size);
#endif
}


#define DEBUG

extern int signal_flag;

void *ground_thread_func(void *arg) {

	struct sockaddr_in air_addr;
	socklen_t addr_len = sizeof(air_addr);
	struct timespec ts;
	unsigned long long air_time_ns;
	unsigned long long ground_time_ns;
	unsigned long long ground_time_ns_network;
	air_timestamp_buffer_t air_timestamps;

	// Recevoir le temps "air" avec timeout
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = TIMEOUT_US;
	setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    while (!signal_flag) {

	#if 1
		if (recvfrom(sockfd, &air_time_ns, sizeof(air_time_ns), 0, (struct sockaddr *)&air_addr, &addr_len) < 0) {
			if (errno == EWOULDBLOCK || errno == EAGAIN) {
				printf("Receive timeout\n");
			} else {
				perror("Failed to receive data");
			}
		}
		else
		{
			air_time_ns = ntohll(air_time_ns);
			// Capturer le temps "ground"
			clock_gettime(CLOCK_MONOTONIC, &ts);
			ground_time_ns = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
		
			// Envoyer le temps "ground" au système "air"
			ground_time_ns_network = htonll(ground_time_ns);
			if (sendto(sockfd, &ground_time_ns_network, sizeof(ground_time_ns_network), 0, (struct sockaddr *)&air_addr, addr_len) < 0) {
				perror("Failed to send data");
			}
			else
			{
		
			#ifdef DEBUG
				//printf("Received air time: %llu us\n", air_time_ns / 1000);
				//printf("Ground time: %llu us\n", ground_time_ns / 1000);
			#endif
				if (recvfrom(sockfd, &air_timestamps, sizeof(air_timestamps), 0, (struct sockaddr *)&air_addr, &addr_len) < 0) {
					if (errno == EWOULDBLOCK || errno == EAGAIN) {
						printf("Receive timeout\n");
					} else {
						perror("Failed to receive data");
					}
				}
				else
				{
					// Convertir les champs en host byte order
					air_timestamps.frameNb = ntohl(air_timestamps.frameNb);
					air_timestamps.vsync_timestamp = ntohll(air_timestamps.vsync_timestamp);
					air_timestamps.framestart_timestamp = ntohll(air_timestamps.framestart_timestamp);
					air_timestamps.frameend_timestamp = ntohll(air_timestamps.frameend_timestamp);
					air_timestamps.ispframedone_timestamp = ntohll(air_timestamps.ispframedone_timestamp);
					air_timestamps.vencdone_timestamp = ntohll(air_timestamps.vencdone_timestamp);
					air_timestamps.one_way_delay_ns = ntohll(air_timestamps.one_way_delay_ns);

					// store it
					//fprintf(stdout, "Packet rcv %lu =>\n", air_timestamps.frameNb);
					//fprintf(stdout, "AirTime:                %llu us\n", air_time_ns / 1000);
					//fprintf(stdout, "GroundTime:             %llu us\n", ground_time_ns / 1000);
					//fprintf(stdout, "Venc Done:              %llu us\n", air_timestamps.vencdone_timestamp / 1000);
					//fprintf(stdout, "Air One Way Delay:      %llu us\n", air_timestamps.one_way_delay_ns / 1000);

					memcpy(&ts_buffers.buffer[air_timestamps.frameNb % K_TS_BUFFER_SIZE].air, &air_timestamps, sizeof(air_timestamp_buffer_t));
					ts_buffers.buffer[air_timestamps.frameNb % K_TS_BUFFER_SIZE].air_time_ns = air_time_ns;
					ts_buffers.buffer[air_timestamps.frameNb % K_TS_BUFFER_SIZE].ground_time_ns = ground_time_ns;

					/*
					fprintf(stdout, "FrameNb: %i Air: %llu, Delay: %llu, Venc: %llu, Ground: %llu\n", 
						air_timestamps.frameNb,
						air_time_ns,
						air_timestamps.one_way_delay_ns,
						air_timestamps.vencdone_timestamp,
						ground_time_ns
					);*/
				}
			}
		}
	#endif
    }
    return NULL;
}

static pthread_t ground_thread;

int timestamp_init(void)
{
	// Créer un socket UDP
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Failed to create socket");
        return 1;
    }

    struct sockaddr_in ground_addr;
    memset(&ground_addr, 0, sizeof(ground_addr));
    ground_addr.sin_family = AF_INET;
    ground_addr.sin_addr.s_addr = INADDR_ANY;
    ground_addr.sin_port = htons(GROUND_PORT);

    // Lier le socket à l'adresse et au port
    if (bind(sockfd, (struct sockaddr *)&ground_addr, sizeof(ground_addr)) < 0) {
        perror("Failed to bind socket");
        close(sockfd);
        return 1;
    }

	// Créer et démarrer le thread pour le système "ground"
	if (pthread_create(&ground_thread, NULL, ground_thread_func, NULL) != 0) {
		perror("Failed to create ground thread");
		close(sockfd);
		return 1;
	}

	return 0;
}

int timestamp_exit(void)
{
	pthread_join(ground_thread, NULL);

    // Fermer le socket à la fin du programme
    close(sockfd);
	return 0;
}

