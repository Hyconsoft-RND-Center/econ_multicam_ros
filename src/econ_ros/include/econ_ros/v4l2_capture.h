#ifndef __V4L2_CAPTURE_H__
#define __V4L2_CAPTURE_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <stdlib.h>
#include "econ_ros/ros_publisher.h"
// #include "gstream.h"

#define CAPTURE_MAX_BUFFER 4
#define MAX_CAPTURE_DEVS 12
#define MAX_DEVICE_NAME 100
#define DEV_NODE "/dev/video%d"
#define MAX_CAM 8
#define UYVY_BYTES_PER_PIXEL 2
#define WIDTH_UHD 3840
#define HEIGHT_UHD 2160
#define WIDTH_UHD_CINEMA 4096
#define HEIGHT_UHD_CINEMA 2160
#define WIDTH_13MP 4192
#define HEIGHT_13MP 3120

extern int show_frame_rate;
extern int sync_flag;
extern short int exit_flag;

enum camera_format_t
{
	YUYV = V4L2_PIX_FMT_YUYV,
	UYVY = V4L2_PIX_FMT_UYVY
};

struct v4l2_capture_buffer
{
	int index;
	void *ptr;
	struct v4l2_buffer buffer;
};

enum loop_state_t
{
	V4L2_CAPTURE_LOOP_PAUSE,
	V4L2_CAPTURE_LOOP_START,
	V4L2_CAPTURE_LOOP_STOP
};

struct v4l2_capture_device_t
{
	int fd;
	uint32_t width;
	uint32_t height;
	uint32_t pix_fmt;
	uint32_t allocation_count;
	struct v4l2_capture_buffer buffer_pool[CAPTURE_MAX_BUFFER];
	enum loop_state_t loop_state;
	pthread_t sync_tid;
	uint32_t temp_sync_val;
};

typedef struct v4l2_capture_device_t v4l2_capture_t;

struct app_data
{
	int capture_dev_count;
	int cameras_connected;
	v4l2_capture_t *camera_dev[MAX_CAPTURE_DEVS];
	PublisherContext *publisher_ctx[MAX_CAPTURE_DEVS];
};

/**
 * @brief : setup a v4l2 capture device
 *
 * @param filename: file name of the device node
 * @param width: width of the frame
 * @param height: height of the frame
 * @param pix_fmt: pixel format
 *
 * @return : valid v4l2_capture device on success
 */
v4l2_capture_t *init_v4l2_capture(const char *filename, uint32_t width, uint32_t height, uint32_t pix_fmt);
int list_resolution(const char *filename);
v4l2_capture_t *init_v4l2_capture(const char *filename, uint32_t width, uint32_t height, uint32_t pix_fmt);
int get_supported_controls(struct app_data *app_data, int cam);
int v4l2_capture_start(v4l2_capture_t *capture_dev);
int v4l2_capture_stop(v4l2_capture_t *capture_dev);
// int get_buffer_size(struct cmdline_args cmdline);
int ctrl_v4l2_parms(struct app_data *app_data, int num_cam, char *capture_mode, int *img_format);
int v4l2_capture_dq_v4l2_buffer(v4l2_capture_t *capture_dev, struct v4l2_buffer *buffer);
int v4l2_capture_q_v4l2_buffer(v4l2_capture_t *capture_dev, struct v4l2_buffer *buffer);
// int v4l2_capture_set_exposure(v4l2_capture_t *capture_dev,int value);
int v4l2_capture_set_sync_all(struct app_data *appdata, int value);
void draw_line();
void draw_single_line();
void print_main_menu();
#endif
