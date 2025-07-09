#include "v4l2_capture.h"
#include <X11/Xlib.h>
#include <getopt.h>
#include <time.h>
#include <glib.h>
#include <math.h>
#include <stdlib.h>
#include <linux/fb.h>
#include "adaptive_application.h"
#include "debug.h"
#include "camera_common.h"
#define FPS 1

#define DEFAULT_FRAMERATE_IN_SYNC 30
#define JETSON_TX1 33
#define JETSON_TX2 24
#define JETSON_XAVIER 25
#define MAX_OVERLAY_TX1 3
#define MAX_OVERLAY_TX2_XVR 6
#define MAX_SYNC_TRY_COUNT 20
#define DISPLAY_DEVNAME_DEFAULT "/dev/fb0"

#define YELLOW "\x1B[33m"
#define WHITE "\x1B[37m"
#define BLUE "\x1B[34m"
#define GREEN "\x1B[32m"
#define RED "\x1B[31m"
#define RESET "\x1B[0m"

#define CLEAR_ENTIRE_LINE "\33[2K"

#define DEFAULT__WIDTH 1920
#define DEFAULT__HEIGHT 1536

#define CAPT_MODE_SINGLE 'c'
#define CAPT_MODE_INDIVIDUAL 'i'
#define CAPT_MODE_DISABLE 'd'

#define RGB888_BYTES_PER_PIXEL 3

#define FRAMES_NOT_IN_SYNC 1
#define FRAMES_IN_SYNC 0

#define CLEAR 0
#define SET 1
#define SINGLE_IMAGE_CAPTURE 1
#define INDIVIDUAL_IMAGE_CAPTURE 2

#define CAMERA_1 0
#define CAMERA_2 1
#define CAMERA_3 2
#define CAMERA_4 3
#define CAMERA_5 4
#define CAMERA_6 5
#define CAMERA_7 6
#define CAMERA_8 7

struct device
{
	char chip_id;
	int overlay_num;
	short int exit_flag;
} device_data;

struct camera_stream
{
	short int num_cam;
	short int dq_err;
	unsigned int display_size;
	unsigned char *framebuffer;
} stream_data;

struct keyboard_event
{
	int active;
	int err;
	pthread_t control_tid;
} key_event_data;

int list_resolution(const char *filename);
guint8 *get_frame_virt(struct app_data *, struct v4l2_buffer *, int);

struct cmdline_args cmdline;

void free_buffer(void *buffer);

guint8 *get_frame_virt(struct app_data *appdata, struct v4l2_buffer *capture_buff, int camera_num);
int tegra_get_chip_id(void);
int get_total_camera_connected(struct app_data *app_data);