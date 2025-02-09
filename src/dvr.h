#ifndef DVR_H
#define DVR_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <queue>
#include <mutex>
#include <condition_variable>

struct MP4E_mux_tag;
struct mp4_h26x_writer_tag;


enum class VideoCodec {
    UNKNOWN=0,
    H264,
    H265
};

static VideoCodec video_codec(const char * str) {
    if (!strcmp(str, "h264")) {
        return VideoCodec::H264;
    }
    if (!strcmp(str, "h265")) {
        return VideoCodec::H265;
    }
    return VideoCodec::UNKNOWN;
}

struct video_params {
    uint32_t video_frm_width;
    uint32_t video_frm_height;
    VideoCodec codec;
};

struct dvr_thread_params {
    char *filename_template;
    int mp4_fragmentation_mode = 0;
    int video_framerate = -1;
    video_params video_p;
};

struct dvr_rpc {
    enum {
        RPC_FRAME,
        RPC_STOP,
        RPC_START,
        RPC_TOGGLE,
        RPC_SHUTDOWN,
        RPC_SET_PARAMS
    } command;
    /* union { */
        std::shared_ptr<std::vector<uint8_t>> frame;
    /*     video_params params; */
    /* }; */
};


extern int dvr_enabled;

class Dvr {
public:
    explicit Dvr(dvr_thread_params params);
    virtual ~Dvr();

    void frame(std::shared_ptr<std::vector<uint8_t>> frame);
    void set_video_params(uint32_t video_frm_width,
                          uint32_t video_frm_height,
                          VideoCodec codec);
    void start_recording();
    void stop_recording();
    void toggle_recording();
    void shutdown();

    static void *__THREAD__(void *context);
private:
    /* void *start(); */
    /* void *stop(); */
    void enqueue_dvr_command(dvr_rpc rpc);

    void loop();
    int start();
    void stop();
    void init();
private:
    std::queue<dvr_rpc> dvrQueue;
    std::mutex mtx;
    std::condition_variable cv;
    char *filename_template;
    int mp4_fragmentation_mode = 0;
    int video_framerate = -1;
    uint32_t video_frm_width;
    uint32_t video_frm_height;
    VideoCodec codec;
    int _ready_to_write = 0;

    FILE *dvr_file;
    MP4E_mux_tag *mux;
    mp4_h26x_writer_tag *mp4wr;
};

#endif
