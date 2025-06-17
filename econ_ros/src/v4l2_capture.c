#include <stdio_ext.h>
#include "debug.h"
#define _ISOC11_SOURCE
#include <stdlib.h>
#include "main.h"
#define READ 0
#define WRITE 1
#define YELLOW "\x1B[33m"
#define WHITE "\x1B[37m"
#define BLUE "\x1B[34m"
#define GREEN "\x1B[32m"
#define RED "\x1B[31m"
#define RESET "\x1B[0m"

#define V4L2_CID_FRAME_SYNC (V4L2_CID_AUTO_FOCUS_RANGE + 12)

int io_bit = 0;
int total_ctrl_supported = 0;
int do_once = 0x3f;
extern int g_skip;

/**
 * @brief: check the capability of the v4l2 device
 *
 * @param fd: file descriptor of the v4l2_device node
 * @param specific_capability: capability to check for
 *
 * @return: if the capability is there then 1 will be returned else zero will be returned
 */

static int check_capability(v4l2_capture_t *capture_dev, int specific_capability)
{
	struct v4l2_capability capability;
	if (ioctl(capture_dev->fd, VIDIOC_QUERYCAP, &capability) < 0)
	{
		ERROR("VIDIOC_QUERYCAP: %s\n", strerror(errno));
		return 0;
	}
	return !!(capability.capabilities & specific_capability);
}

/**
 * @brief : get v4l2 capture format
 *
 * @param capture_dev: capture device
 * @param fmt: fmt
 *
 * @return : 0 on success
 */

static int get_v4l2_capture_format(v4l2_capture_t *capture_dev, struct v4l2_format *fmt)
{
	memset(fmt, 0, sizeof(struct v4l2_format));
	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(capture_dev->fd, VIDIOC_G_FMT, fmt) < 0)
	{
		ERROR("VIDIO_G_FMT: %s\n", strerror(errno));
		return -1;
	}

	DEBUG(3, "Type %d Format: %d Width: %d Height: %d\n",
		  fmt->type, fmt->fmt.pix.pixelformat, fmt->fmt.pix.width,
		  fmt->fmt.pix.height);
	return 0;
}

/**
 * @brief : set v4l2 capture format
 *
 * @param capture_dev: capture device
 * @param fmt: v4l2 format
 *
 * @return : 0 on success
 */

static int set_v4l2_capture_format(v4l2_capture_t *capture_dev, struct v4l2_format *fmt)
{
	if (ioctl(capture_dev->fd, VIDIOC_S_FMT, fmt) < 0)
	{
		ERROR("VIDIOC_S_FMT: %s \n", strerror(errno));
		return -errno;
	}

	capture_dev->width = fmt->fmt.pix.width;
	capture_dev->height = fmt->fmt.pix.height;
	capture_dev->pix_fmt = fmt->fmt.pix.pixelformat;
	DEBUG(3, "Func %s Type %d Format: %d Width: %d Height: %d\n", __func__,
		  fmt->type, fmt->fmt.pix.pixelformat, fmt->fmt.pix.width,
		  fmt->fmt.pix.height);
	return 0;
}

/**
 * @brief : set format information for capture device
 *
 * @param capture_dev: camera device
 *
 * @return : 0 on success
 */

static int set_format_info(v4l2_capture_t *capture_dev)
{
	struct v4l2_format fmt;
	memset(&fmt, 0, sizeof(struct v4l2_format));

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	get_v4l2_capture_format(capture_dev, &fmt);

	fmt.fmt.pix.pixelformat = capture_dev->pix_fmt;
	fmt.fmt.pix.height = capture_dev->height;
	fmt.fmt.pix.width = capture_dev->width;
	fmt.fmt.pix.sizeimage = (capture_dev->width * capture_dev->height) * 2;
	DEBUG(3, "Func %s Type %d Format: %d Width: %d Height: %d\n", __func__,
		  fmt.type, fmt.fmt.pix.pixelformat, fmt.fmt.pix.width,
		  fmt.fmt.pix.height);
	// set_framerate(capture_dev,30);
	set_v4l2_capture_format(capture_dev, &fmt);

	get_v4l2_capture_format(capture_dev, &fmt);

	return 0;
}

/**
 * @brief : alloc capture buffer using user pointer method
 *
 * @param capture_dev: capture device
 *
 * @return : 0 on success
 */

int alloc_capture_buffers_userptr(v4l2_capture_t *capture_dev)
{
	struct v4l2_requestbuffers reqbuf;
	struct v4l2_buffer buf;
	long sz = sysconf(_SC_PAGESIZE);
	uint32_t i;
	memset(&reqbuf, 0, sizeof(struct v4l2_requestbuffers));
	memset(&buf, 0, sizeof(struct v4l2_buffer));

	reqbuf.count = capture_dev->allocation_count;
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqbuf.memory = V4L2_MEMORY_USERPTR;

	if (ioctl(capture_dev->fd, VIDIOC_REQBUFS, &reqbuf) < 0)
	{
		ERROR("VIDIOC_REQBUFS: %s\n", strerror(errno));
		return -1;
	}

	for (i = 0; i < capture_dev->allocation_count; i++)
	{
		capture_dev->buffer_pool[i].index = i;
		capture_dev->buffer_pool[i].ptr = aligned_alloc(sz, capture_dev->width * capture_dev->height * 2);
		capture_dev->buffer_pool[i].buffer.type = reqbuf.type;
		capture_dev->buffer_pool[i].buffer.index = i;
		capture_dev->buffer_pool[i].buffer.length = capture_dev->width * capture_dev->height * 2;
		capture_dev->buffer_pool[i].buffer.m.userptr = (unsigned long)capture_dev->buffer_pool[i].ptr;
		capture_dev->buffer_pool[i].buffer.memory = V4L2_MEMORY_USERPTR;
		if (ioctl(capture_dev->fd, VIDIOC_QBUF, &capture_dev->buffer_pool[i].buffer) < 0)
		{
			ERROR("VIDIOC_QBUF: %s\n", strerror(errno));
		}
		DEBUG(3, "index : %d address: %p \n", i, capture_dev->buffer_pool[i].ptr);
	}
	return 0;
}

/**
 * @brief : set manual exposure for v4l2 capture device
 *
 * @param capture_dev: capture device
 * @param value: value
 *
 * @return : 0 on success
 */

int v4l2_capture_set_exposure(v4l2_capture_t *capture_dev, int value)
{
	struct v4l2_control ctrl;
	memset(&ctrl, 0, sizeof(struct v4l2_control));

	ctrl.id = V4L2_CID_EXPOSURE_AUTO;
	ctrl.value = 1;
	if (ioctl(capture_dev->fd, VIDIOC_S_CTRL, &ctrl) < 0)
	{
		ERROR("unable to set exposure: %s\n", strerror(errno));
		return -1;
	}

	ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
	ctrl.value = value;
	if (ioctl(capture_dev->fd, VIDIOC_S_CTRL, &ctrl) < 0)
	{
		ERROR("unable to set exposure : %s\n", strerror(errno));
		return -1;
	}

	return 0;
}

void *init_sync(void *data)
{
	v4l2_capture_t *capture_dev = (v4l2_capture_t *)data;
	if (v4l2_capture_set_sync(capture_dev, capture_dev->temp_sync_val) < 0)
	{
		ERROR(" %s - Setting sync failed value = %d \n", __func__, capture_dev->temp_sync_val);
	}
	pthread_exit(NULL);
}

int v4l2_capture_set_sync_all(struct app_data *app_data, int value)
{
	int i = 0;
	for (i = 0; i < app_data->cameras_connected; i++)
	{
		/* NEED to call sync in a thread and then wait till join */
		app_data->camera_dev[i]->temp_sync_val = value;
		if (pthread_create(&app_data->camera_dev[i]->sync_tid, NULL,
						   &init_sync, (void *)app_data->camera_dev[i]) < 0)
		{
			ERROR("unable to create sync thread: %s\n", strerror(errno));
		}
	}

	for (i = 0; i < app_data->cameras_connected; i++)
	{
		/* NEED to call sync in a thread and then wait till join */
		if (pthread_join(app_data->camera_dev[i]->sync_tid, NULL) < 0)
		{
			ERROR("unable to join sync thread: %s\n", strerror(errno));
		}
	}
	return 0;
}

int v4l2_capture_set_sync(v4l2_capture_t *capture_dev, int value)
{
	struct v4l2_control ctrl;
	memset(&ctrl, 0, sizeof(struct v4l2_control));

	ctrl.id = V4L2_CID_FRAME_SYNC;
	ctrl.value = value;
	if (ioctl(capture_dev->fd, VIDIOC_S_CTRL, &ctrl) < 0)
	{
		ERROR("unable to set sync: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

/**
 * @brief : change the capture loop state to start
 *
 * @param capture_dev: capture device
 *
 * @return : 0 on success
 */
int v4l2_capture_start(v4l2_capture_t *capture_dev)
{
	int buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(capture_dev->fd, VIDIOC_STREAMON, &buffer_type) < 0)
	{
		ERROR("VIDIOC_STREAMON: %s\n", strerror(errno));
	}
	return 0;
}

int v4l2_capture_stop(v4l2_capture_t *capture_dev)
{
	int buffer_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(capture_dev->fd, VIDIOC_STREAMOFF, &buffer_type) < 0)
	{
		ERROR("VIDIOC_STREAMON: %s\n", strerror(errno));
	}
	return 0;
}

int v4l2_capture_dq_v4l2_buffer(v4l2_capture_t *capture_dev, struct v4l2_buffer *buffer)
{
	buffer->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(capture_dev->fd, VIDIOC_DQBUF, buffer) < 0)
	{
		ERROR("VIDIOC_DQBUF: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

int v4l2_capture_q_v4l2_buffer(v4l2_capture_t *capture_dev, struct v4l2_buffer *buffer)
{
	buffer->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(capture_dev->fd, VIDIOC_QBUF, buffer) < 0)
	{
		if (frame_sync_data.sync_flag != 0)
			ERROR("VIDIOC_QBUF: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

int list_resolution(const char *filename)
{
	struct v4l2_fmtdesc *fmt = NULL;
	struct v4l2_frmsizeenum *frmsize = NULL;
	int fd;
	fd = open(filename, O_RDWR);

	if (fd < 0)
	{
		ERROR("%s -> %s \n", filename, strerror(errno));
		return 1;
	}

	fmt = malloc(sizeof(struct v4l2_format) * 1);

	if (!fmt)
	{
		printf("memory allocation failed");
		return 1;
	}

	fmt->index = 0; // index 0
	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (ioctl(fd, VIDIOC_ENUM_FMT, fmt) >= 0)
		printf("\nsupported format %s \n", fmt->description);
	else
		printf("ENUM_FMT failed\n");

	frmsize = malloc(sizeof(struct v4l2_frmsizeenum) * 1);
	if (frmsize == NULL)
	{
		printf("memory allocation failed");
		return 1;
	}

	frmsize->pixel_format = fmt->pixelformat;
	frmsize->index = 0;

	while (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, (frmsize)) >= 0)
	{
		if (frmsize->type == V4L2_FRMSIZE_TYPE_DISCRETE && frmsize->discrete.width != WIDTH_13MP && frmsize->discrete.height != HEIGHT_13MP && !(frmsize->discrete.width == WIDTH_UHD_CINEMA && frmsize->discrete.height == HEIGHT_UHD_CINEMA))
			printf("\n %d: width=%d height=%d\n", frmsize->index, frmsize->discrete.width, frmsize->discrete.height);
		else if (frmsize->type == V4L2_FRMSIZE_TYPE_STEPWISE &&
				 frmsize->stepwise.max_width != WIDTH_13MP && frmsize->stepwise.max_height != HEIGHT_13MP)
			printf("\n %d: width=%d height=%d\n", frmsize->index, frmsize->stepwise.max_width, frmsize->stepwise.max_height);

		frmsize->index++;
	}
	free(fmt);
	free(frmsize);
	fmt = NULL;
	frmsize = NULL;
	close(fd);
	return 0;
}
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

v4l2_capture_t *init_v4l2_capture(const char *filename, uint32_t width, uint32_t height, uint32_t pix_fmt)
{
	struct v4l2_frmsizeenum frmsize;
	short int c_width, c_height, res_found = 0;
	v4l2_capture_t *capture_dev = (v4l2_capture_t *)malloc(sizeof(v4l2_capture_t));

	if (capture_dev == NULL)
	{
		ERROR("malloc: capture_dev -> %s\n", filename);
		return NULL;
	}

	/* open camera device */
	capture_dev->fd = open(filename, O_RDWR);
	if (capture_dev->fd < 0)
	{
		ERROR("%s -> %s \n", filename, strerror(errno));
		goto err_exit;
	}

	frmsize.pixel_format = pix_fmt;
	frmsize.index = 0;

	while (ioctl(capture_dev->fd, VIDIOC_ENUM_FRAMESIZES, (&frmsize)) >= 0)
	{
		if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
		{
			c_width = frmsize.discrete.width;
			c_height = frmsize.discrete.height;
		}
		else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
		{
			c_width = frmsize.stepwise.max_width;
			c_height = frmsize.stepwise.max_height;
		}
		if (width == c_width && height == c_height)
		{
			res_found = 1;
			break;
		}
		frmsize.index++;
	}
	if (res_found == 0)
	{
		printf("RESOLUTION NOT SUPPORTED\n");
		goto err_exit;
	}
	capture_dev->width = width;
	capture_dev->height = height;
	capture_dev->pix_fmt = pix_fmt;
	capture_dev->allocation_count = CAPTURE_MAX_BUFFER;

	if (!check_capability(capture_dev, V4L2_CAP_STREAMING))
	{
		ERROR("Device not capable for streaming\n");
		goto err_exit;
	}

	set_format_info(capture_dev);
#if 1
	int fd_pid = 0;
	int app_pid = getpid();
	char *mypid = malloc(10); // ex. 34567
	sprintf(mypid, "%d", app_pid);

	if ((fd_pid = open("/sys/kernel/xerror_file/get_pid_from_application", O_RDWR)) < 0)
	{

		printf("unable to open file to write pid\n");
	}
	if ((write(fd_pid, &mypid[0], 10)) < 0)
	{

		printf("UNABLE TO write fo the given file\n");
	}
	close(fd_pid);

#endif

	alloc_capture_buffers_userptr(capture_dev);
	capture_dev->loop_state = V4L2_CAPTURE_LOOP_PAUSE;
	return capture_dev;
err_exit:
	close(capture_dev->fd);
	free(capture_dev);
	return NULL;
}

struct v4l2_queryctrl supported_ctrl[20];
struct v4l2_queryctrl white_balamce_ctrl[3];
struct v4l2_queryctrl exposure_ctrl[2];

static int query_ioctl(int hdevice, int current_ctrl, struct v4l2_queryctrl *ctrl)
{
	int ret = 0;
	int tries = 4;
	do
	{
		if (ret)
			ctrl->id = current_ctrl | V4L2_CTRL_FLAG_NEXT_CTRL;
		ret = ioctl(hdevice, VIDIOC_QUERYCTRL, ctrl);
	} while (ret && tries-- &&
			 ((errno == EIO || errno == EPIPE || errno == ETIMEDOUT)));

	return (ret);
}

int get_supported_controls(struct app_data *app_data, int cam)
{
	int total_wb_supprted = 0;
	int total_exposure_supported = 0;
	struct v4l2_queryctrl qctrl;
	int fd = app_data->camera_dev[cam]->fd;
	int currentctrl = 0;
	qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;

	while ((query_ioctl(fd, currentctrl, &qctrl)) == 0)
	{
		if (!((!strcmp((char *)qctrl.name, "Bypass Mode")) || (!strcmp((char *)qctrl.name, "User Controls")) || (!strcmp((char *)qctrl.name, "Camera Controls")) || (!strcmp((char *)qctrl.name, "Override Enable")) || (!strcmp((char *)qctrl.name, "Height Align")) || (!strcmp((char *)qctrl.name, "Low Latency Mode")) || (!strcmp((char *)qctrl.name, "Sensor Modes")) || (!strcmp((char *)qctrl.name, "Write ISP format")) || (!strcmp((char *)qctrl.name, "Preferred Stride")) || (!strcmp((char *)qctrl.name, "Size Align"))))
		{
			if ((qctrl.id != V4L2_CID_EXPOSURE_AUTO) && (qctrl.id != V4L2_CID_AUTO_WHITE_BALANCE))
			{
				supported_ctrl[total_ctrl_supported].id = qctrl.id;
				supported_ctrl[total_ctrl_supported].type = qctrl.type;
				memcpy(supported_ctrl[total_ctrl_supported].name, qctrl.name, 32);
				supported_ctrl[total_ctrl_supported].minimum = qctrl.minimum;
				supported_ctrl[total_ctrl_supported].maximum = qctrl.maximum;
				supported_ctrl[total_ctrl_supported].step = qctrl.step;
				supported_ctrl[total_ctrl_supported].default_value = qctrl.default_value;
				supported_ctrl[total_ctrl_supported].flags = qctrl.flags;
				total_ctrl_supported++;
			}

			if ((qctrl.id == V4L2_CID_AUTO_WHITE_BALANCE) || (qctrl.id == V4L2_CID_WHITE_BALANCE_TEMPERATURE))
			{
				white_balamce_ctrl[total_wb_supprted].id = qctrl.id;
				white_balamce_ctrl[total_wb_supprted].type = qctrl.type;
				memcpy(white_balamce_ctrl[total_wb_supprted].name, qctrl.name, 32);
				white_balamce_ctrl[total_wb_supprted].minimum = qctrl.minimum;
				white_balamce_ctrl[total_wb_supprted].maximum = qctrl.maximum;
				white_balamce_ctrl[total_wb_supprted].step = qctrl.step;
				white_balamce_ctrl[total_wb_supprted].default_value = qctrl.default_value;
				white_balamce_ctrl[total_wb_supprted].flags = qctrl.flags;
				total_wb_supprted++;
			}

			if ((qctrl.id == V4L2_CID_EXPOSURE_AUTO) || (qctrl.id == V4L2_CID_EXPOSURE_ABSOLUTE))
			{
				exposure_ctrl[total_exposure_supported].id = qctrl.id;
				exposure_ctrl[total_exposure_supported].type = qctrl.type;
				memcpy(exposure_ctrl[total_exposure_supported].name, qctrl.name, 32);
				exposure_ctrl[total_exposure_supported].minimum = qctrl.minimum;
				exposure_ctrl[total_exposure_supported].maximum = qctrl.maximum;
				exposure_ctrl[total_exposure_supported].step = qctrl.step;
				exposure_ctrl[total_exposure_supported].default_value = qctrl.default_value;
				exposure_ctrl[total_exposure_supported].flags = qctrl.flags;
				total_exposure_supported++;
			}
		}
		currentctrl = qctrl.id;

		qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
	}

	if (total_ctrl_supported == 0)
		return -1;
	return 0;
}

int control(int fd, struct v4l2_control *ctrl)
{
	if (io_bit == WRITE)
	{
		if (ioctl(fd, VIDIOC_S_CTRL, ctrl) < 0)
		{
			perror("CAPTURE: VIDIOC_S_CTRL");
			return -1;
		}
	}
	else if (io_bit == READ)
	{
		if (ioctl(fd, VIDIOC_G_CTRL, ctrl) < 0)
		{
			printf("CAPTURE: VIDIOC_G_CTRL");
			return -1;
		}
	}
	return 0;
}

void list_camera(int num_cam)
{
	int i;
	printf("\n");
	draw_line();
	printf(GREEN "|\t\tSELECT AVAILABLE DEVICE TO CHANGE CONTROLS\t     |\n");
	draw_line();
	printf(YELLOW "\tCamera" WHITE " \t\t|\t" GREEN "Device node\t\t\n");
	draw_single_line();

	for (i = 0; i < MAX_CAM; i++)
	{
		if (num_cam & (1 << i))
		{
			printf(YELLOW "\t %d", i);
			printf(WHITE "\t\t|" GREEN "\t/dev/video%d\n" RESET, i);
			draw_single_line();
		}
	}
	printf(YELLOW "\t 8\t\t|" GREEN "\tCapture all frames in a single image\n");
	draw_single_line();
	printf(YELLOW "\t 9\t\t|" GREEN "\tCapture all frames as individual images\n");
	draw_single_line();
	printf(YELLOW "\t 10\t\t|" GREEN "\tExit from Application\n");
	draw_single_line();
	printf(WHITE "Enter the required " YELLOW "Camera :: \n\n" RESET);
}

int image_format_option()
{
	char input[16];
	int format;
	draw_line();
	printf(GREEN "\t\tImage formats Available\n");
	draw_line();
	printf(YELLOW "\t 1\t\t|" GREEN "\t Raw Image Format\n");
	draw_line();
	printf(YELLOW "\t 2\t\t|" GREEN "\t BMP Image Format\n");
	draw_line();
	printf(YELLOW "\t 3\t\t|" GREEN "\t JPEG Image Format\n");
	draw_line();
	printf("\nEnter the required format:\n");
	scanf("%s", input);
	format = atoi(input);
	if (format < 1 || format > 3)
	{
		printf(RED "\nInvalid choice of image format\n" RESET);
		format = image_format_option(format);
	}
	return format;
}

int ctrl_v4l2_parms(struct app_data *app_data, int num_cam, char *capture_mode, int *img_format)
{
	int fd, fd1;
	int cam, i, option;
	int menu_option = 0;
	char scan_string[16];
	int control_index = 0;
	int exit_option = 0;
	int fps_option = 0;
	int sub_menu_option = 0;
	struct v4l2_control ctrl;
	char input[3];
	printf("\v\v\v\v\v");
	print_main_menu();
	for (;;)
	{
		do
		{
			__fpurge(stdin);
			list_camera(num_cam);
			fgets(input, 3, stdin);
			option = atoi(input);
			if (option < 0 && option > 10)
				cam = 0xff;
			else
				cam = atoi(input);
			draw_line();
			if ((num_cam & (1 << cam)))
				printf("You have selected Camera %d\n", cam);
			else if (cam == 8)
			{
				printf("\nSingle image capture selected\n");
				*img_format = image_format_option(img_format);
				*capture_mode = 'c';
				sleep(2);
			}
			else if (cam == 9)
			{
				printf("\nIndividual Image capture selected\n");
				*img_format = image_format_option(img_format);
				*capture_mode = 'i';
				sleep(2);
			}
			else if (cam == 10)
			{
				printf(GREEN "\nExited from Application\n");
				device_data.exit_flag = 1;
				sleep(2);
			}
			else
				printf("camera not available\n");
			draw_line();
		} while (!(num_cam & (1 << cam)));

		fd = app_data->camera_dev[cam]->fd;

		for (;;)
		{
			printf("\n");
			draw_line();

			printf(GREEN "\t\tImage Controls Camera %d\n", cam);
			draw_line();
			printf(YELLOW "Control\t" WHITE "|" GREEN " DESCRIPTION\n" RESET);
			draw_single_line();

			for (control_index = 0; control_index < total_ctrl_supported; control_index++)
				printf(YELLOW "%d" WHITE "\t|" GREEN " %s\n" RESET, control_index + 1, supported_ctrl[control_index].name);

			fps_option = total_ctrl_supported + 1;
			exit_option = fps_option + 1;
			printf(YELLOW "%d ", fps_option);
			printf(WHITE "\t|" GREEN " Display average frame rate \n");
			printf(YELLOW "%d", exit_option);
			printf(WHITE "\t|" GREEN " Exit from Features Menu \n" RESET);
			draw_single_line();
			printf("Enter the " YELLOW "Control: \n" RESET);
			scanf("%s", scan_string);
			menu_option = atoi(scan_string);
			control_index = menu_option - 1;

			if ((control_index < 0) || (control_index > exit_option))
				printf("Wrong option: Enter correct option \n");
			else if (menu_option == exit_option)
				break;
			else if (menu_option == fps_option)
			{
				for (;;)
				{
					draw_line();
					printf(" Show Frame rate debug message                          \n");
					draw_line();
					printf(YELLOW "Value\t" WHITE "|" GREEN " DESCRIPTION\n" RESET);
					draw_single_line();
					printf(YELLOW " 0" WHITE "\t|" GREEN " Disable                                            \n" RESET);
					printf(YELLOW " 1" WHITE "\t|" GREEN " Enable                                             \n" RESET);
					printf(YELLOW " 2" WHITE "\t|" GREEN " Exit from Show Frame rate menu                     \n" RESET);
					draw_single_line();
					printf(" Note:\n");
					printf(" Current Value is %d \n", frame_rate_data.show_frame_rate);
					draw_line();
					printf(" Enter the " YELLOW "Value : \n" RESET);
					scanf("%s", scan_string);
					menu_option = atoi(scan_string);

					if (menu_option == 2)
					{
						break;
					}
					else
					{
						switch (menu_option)
						{
						case 0:
						case 1:
							frame_rate_data.show_frame_rate = menu_option;
							break;
						default:
						{
							printf("\nError in input!, Please choose correct option \n");
						}
							continue;
						}
					}
				}
			}
			else if (menu_option <= total_ctrl_supported)
			{
				switch (supported_ctrl[control_index].id)
				{
				case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
					for (;;)
					{
						draw_line();
						printf(GREEN " WhiteBalance Menu \n");
						draw_line();
						printf(YELLOW "Value\t" WHITE "|" GREEN " DESCRIPTION\n" RESET);
						draw_single_line();
						printf(YELLOW " 1" WHITE "\t|" GREEN " %s \n", white_balamce_ctrl[0].name);
						printf(YELLOW " 2" WHITE "\t|" GREEN " %s \n", white_balamce_ctrl[1].name);
						printf(YELLOW " 3" WHITE "\t|" GREEN " Exit from whitebalance  menu      \n" RESET);
						draw_line();
						printf("\n  Please choose the " YELLOW "Value :\n" RESET);

						scanf("%s", scan_string);
						menu_option = atoi(scan_string);

						control_index = menu_option - 1;
						if ((menu_option < 1) || (menu_option > 3))
							printf(WHITE "Wrong option: Enter proper option \n");
						else if (menu_option == 3)
							break;
						else
						{
							draw_line();
							printf(GREEN "%s Menu -- Range %d to %d, Step Size = %d, Default Value = %d \n", white_balamce_ctrl[control_index].name, white_balamce_ctrl[control_index].minimum,
								   white_balamce_ctrl[control_index].maximum, white_balamce_ctrl[control_index].step, white_balamce_ctrl[control_index].default_value);
							draw_line();

							ctrl.id = white_balamce_ctrl[control_index].id;
							io_bit = READ;
							if (control(fd, &ctrl) < 0)
							{
								printf("\n Unable to READ the control. Please try again \n");
								continue;
							}
							printf("Current %s Value is %d\n", white_balamce_ctrl[control_index].name, ctrl.value);
							draw_line();
							printf("Please enter the %s Value : \n", white_balamce_ctrl[control_index].name);
							scanf("%s", scan_string);
							ctrl.value = atoi(scan_string);

							if ((ctrl.value < white_balamce_ctrl[control_index].minimum) || (ctrl.value > white_balamce_ctrl[control_index].maximum))
							{
								printf(" Enter input within the range (%d to %d)\n", white_balamce_ctrl[control_index].minimum, white_balamce_ctrl[control_index].maximum);
								continue;
							}
							ctrl.id = white_balamce_ctrl[control_index].id;
							io_bit = WRITE;
							if (control(fd, &ctrl) < 0)
							{
								printf("\n Unable to WRITE the control. Please try again \n");
								continue;
							}
						}
					}
					break;
				case V4L2_CID_EXPOSURE_ABSOLUTE:
				{
					for (;;)
					{
						draw_line();
						printf(GREEN " Exposure Menu \n");
						draw_line();
						printf(YELLOW "Value\t" WHITE "|" GREEN " DESCRIPTION\n" RESET);
						draw_single_line();
						printf(YELLOW " 1" WHITE "\t|" GREEN " Exposure Mode \n");
						printf(YELLOW " 2" WHITE "\t|" GREEN " %s \n", exposure_ctrl[1].name);
						printf(YELLOW " 3" WHITE "\t|" GREEN " Exit from exposure  menu      \n" RESET);
						draw_line();
						printf("\n  Please choose the " YELLOW "Value :\n" RESET);
						scanf("%s", scan_string);
						menu_option = atoi(scan_string);
						control_index = menu_option - 1;
						if ((menu_option < 1) || (menu_option > 3))
							printf("Wrong option: Enter proper option \n");
						else if (menu_option == 3)
							break;
						else
						{
							draw_line();
							printf(GREEN "Exposure Menu -- Range %d to %d, Step Size = %d Default Value = %d\n", exposure_ctrl[control_index].minimum, exposure_ctrl[control_index].maximum, exposure_ctrl[control_index].step, exposure_ctrl[control_index].default_value);
							draw_line();

							io_bit = READ;
							ctrl.id = exposure_ctrl[control_index].id;
							if (control(fd, &ctrl) < 0)
							{
								printf("\n Unable to READ the control. Please try again \n");
								continue;
							}
							printf("Current Exposure Mode Value is %d\n", ctrl.value);
							draw_line();
							printf("Please enter the Exposure Mode Value : \n");
							scanf("%s", scan_string);
							ctrl.value = atoi(scan_string);

							if ((ctrl.value < exposure_ctrl[control_index].minimum) || (ctrl.value > exposure_ctrl[control_index].maximum))
							{
								printf(" Enter input within the range (%d to %d)\n", exposure_ctrl[control_index].minimum, exposure_ctrl[control_index].maximum);
								continue;
							}
							io_bit = WRITE;
							ctrl.id = exposure_ctrl[control_index].id;
							if (control(fd, &ctrl) < 0)
							{
								printf("\n Unable to WRITE the control. Please try again \n");
								continue;
							}
						}
					}
				}
				break;
				default:
					for (;;)
					{
						sub_menu_option = 1;
						draw_line();
						printf(GREEN "%s Menu -- Range %d to %d, Step Size = %d, Default Value = %d \n", supported_ctrl[control_index].name, supported_ctrl[control_index].minimum,
							   supported_ctrl[control_index].maximum, supported_ctrl[control_index].step, supported_ctrl[control_index].default_value);
						draw_line();
						printf(YELLOW "Value\t" WHITE "|" GREEN " DESCRIPTION\n" RESET);
						draw_single_line();
						printf(YELLOW " %d" WHITE "\t|" GREEN " Choose %s level \n", sub_menu_option++, supported_ctrl[control_index].name);
						printf(YELLOW " %d" WHITE "\t|" GREEN " Exit from  %s menu \n" RESET, sub_menu_option++, supported_ctrl[control_index].name);
						printf(" Note:\n");

						io_bit = READ;
						ctrl.id = supported_ctrl[control_index].id;
						if (control(fd, &ctrl) < 0)
						{
							printf("\n Unable to READ the control. Please try again \n");
							continue;
						}
						printf("Current %s Value is %d\n", supported_ctrl[control_index].name, ctrl.value);
						draw_line();
						printf("\n  Please choose the " YELLOW "Value :\n" RESET);
						scanf("%s", scan_string);
						sub_menu_option = atoi(scan_string);

						if (sub_menu_option == 2)
							break;
						switch (sub_menu_option)
						{
						case 1:
							printf("Please enter the %s Value : \n", supported_ctrl[control_index].name);
							scanf("%s", scan_string);
							ctrl.value = atoi(scan_string);

							if ((ctrl.value < supported_ctrl[control_index].minimum) || (ctrl.value > supported_ctrl[control_index].maximum))
							{
								printf(" Enter input within the range (%d to %d)\n", supported_ctrl[control_index].minimum, supported_ctrl[control_index].maximum);
								continue;
							}
							io_bit = WRITE;
							ctrl.id = supported_ctrl[control_index].id;
							if (ctrl.id == V4L2_CID_FRAME_SYNC)
							{
								/* NEED to call sync in a thread and then wait till join */
								v4l2_capture_set_sync_all(app_data, ctrl.value);
								for (i = 0; i < app_data->cameras_connected; i++)
								{
									fd1 = app_data->camera_dev[i]->fd;
									if (control(fd1, &ctrl) < 0)
									{
										printf("\n Unable to WRITE the control. Please try again \n");
									}
									if (ctrl.value == 0)
										frame_sync_data.sync_flag = 0;
									if (ctrl.value == 1)
									{
										frame_sync_data.sync_flag |= (1 << i);
										frame_sync_data.g_skip = 0;
									}
								}
							}
							else
							{
								if (control(fd, &ctrl) < 0)
								{
									printf("\n Unable to WRITE the control. Please try again \n");
									continue;
								}
							}
							break;
						case 2:
							break;
						default:
							printf("\nError in input!, Please choose correct option \n");
						}
					}
					break;
				}
			}
		}
	}
	return 0;
}
