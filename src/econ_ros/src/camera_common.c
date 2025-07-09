#include "main.h"
#include "v4l2_capture.h"

#define SYNC_TOLERANCE_US 1000

/* Change camera controls */
void *init_control(void *data)
{
	int cam = 0;
	while (!(stream_data.num_cam & (1 << cam)))
	{
		cam++;
	}
	if (cmdline.record != 1)
	{
		get_supported_controls((struct app_data *)data, cam);
		// ctrl_v4l2_parms((struct app_data *)data, stream_data.num_cam,
		// &img_common_data.capture_mode, &img_common_data.img_format);
	}
	return 0;
}

void init_app_data(void)
{
	frame_sync_data.sync_flag = 0xff;
	frame_sync_data.fsyn = FRAMES_IN_SYNC;

	img_common_data.capture_mode = CAPT_MODE_DISABLE;

	key_event_data.active = 1;

	stream_data.dq_err = 0xff;
}

int get_buffer_size(struct cmdline_args cmdline)
{
	return (cmdline.width * cmdline.height * UYVY_BYTES_PER_PIXEL);
}

int init_buffer_size(struct cmdline_args cmdline)
{
	img_common_data.buffer_size = get_buffer_size(cmdline);

	// Allocating the length of buffer based on command line inputs
	img_common_data.save_image_size = cmdline.width * cmdline.height * UYVY_BYTES_PER_PIXEL;

	return 0;
}
int v4l2_start_stream(struct app_data *app_data, struct cmdline_args *cmdline)
{
	char *dev_node = (char *)malloc((sizeof(char)) * 16);
	int cam = 0, num_cam = 0;
	for (cam = 0; cam < app_data->cameras_connected; cam++)
	{
		sprintf(dev_node, DEV_NODE, cam);
		app_data->camera_dev[cam] = init_v4l2_capture(dev_node, cmdline->width, cmdline->height, UYVY);
		if (app_data->camera_dev[cam] != NULL)
			num_cam |= (1 << cam);
	}
	/* NEED to call sync in a thread and then wait till join */
	v4l2_capture_set_sync_all(app_data, 0);
	if (frame_sync_data.sync_flag != 0)
		v4l2_capture_set_sync_all(app_data, 1);

	free(dev_node);

	if (num_cam == 0)
		return 0;
	else
	{
		for (cam = 0; cam < app_data->cameras_connected; cam++)
		{
			if (num_cam & (1 << cam))
			{
				v4l2_capture_start(app_data->camera_dev[cam]);
			}
		}
	}
	return num_cam;
}

int init_v4l2_camera(struct app_data *app_data, struct cmdline_args *cmdline, struct v4l2_buffer *camera_buffer)
{
	int cam = 0;
	/* Getting the Display resolution */
	// if (get_display_resolution(cmdline)) {
	// 	fprintf(stderr, "Error opening display \n");
	// 	return -1;
	// }
	/* V4L2 memory and device type  */
	for (cam = 0; cam < app_data->cameras_connected; cam++)
	{
		(camera_buffer + cam)->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		(camera_buffer + cam)->memory = V4L2_MEMORY_USERPTR;
	}

	stream_data.num_cam = v4l2_start_stream(app_data, cmdline);

	if (stream_data.num_cam == 0)
	{
		fprintf(stderr, "Failed to init_v4l2_capture \n");
		return -1;
	}

	return 0;
}

// int init_gstreamer_handler( struct app_data *app_data, struct cmdline_args *cmdline, struct gst_handler *gst_handle[])
// {
// 	gst_handle[0] = (struct gst_handler *)malloc(sizeof(struct gst_handler));
// 	if (gst_handle[0] == NULL) {
// 		fprintf(stderr, "cannot allocate memory for gst handle\n");
// 		return -1;
// 	}
// 	if (cmdline->record != 1 )
// 		create_gst_handle(gst_handle[0], app_data->cameras_connected, 0);
// 	else
// 		create_record_handle(gst_handle[0], app_data->cameras_connected);
// 	start_stream(gst_handle[0]);

// 	/* For BackGround Filling */
// 	if (cmdline->record != 1) {
// 		gst_handle[1] = (struct gst_handler *)malloc(sizeof(struct gst_handler));
// 		if (gst_handle[1] == NULL) {
// 			fprintf(stderr, "cannot allocate memory for gst handle\n");
// 			free(app_data);
// 			return -1;
// 		}
// 		// as using nvcompositor, no need background frame
// 		create_gst_handle(gst_handle[1], app_data->cameras_connected, 1);
// 		//start_stream(gst_handle[1]);

// 		stream_data.display_size = cmdline->d_width * cmdline->d_height * UYVY_BYTES_PER_PIXEL;
// 	}
// 	return 0;
// }

int get_camera_frames(struct app_data *app_data, struct v4l2_buffer *buffer)
{
	int cam = 0;
	int ret = 0;
	int count = 0;
	int cam_sync = 0;
	int retry = 0;

	unsigned long int usec[MAX_CAM], high_usec = 0, low_usec = 0;
	for (cam = 0; cam < app_data->cameras_connected; cam++)
	{
		memset(buffer + cam, 0, sizeof(struct v4l2_buffer));
	}
	/* spiking 2 frames for Catchup camera frame with PWM chip */
	while (frame_sync_data.g_skip <= 5)
	{
		for (cam = 0; cam < app_data->cameras_connected; cam++)
		{
			if (stream_data.num_cam & (1 << cam))
			{
				v4l2_capture_dq_v4l2_buffer(app_data->camera_dev[cam], buffer + cam);
				v4l2_capture_q_v4l2_buffer(app_data->camera_dev[cam], buffer + cam);
			}
		}
		frame_sync_data.g_skip++;
	}

	for (count = 0; count < MAX_SYNC_TRY_COUNT; count++)
	{
		// Set highest and lowest timestamp values for all frames
		for (cam = 0; cam < app_data->cameras_connected; cam++)
		{
			if ((stream_data.num_cam & (1 << cam)) && (stream_data.dq_err & (1 << cam)))
			{
				ret = v4l2_capture_dq_v4l2_buffer(app_data->camera_dev[cam], buffer + cam);
				if (((buffer + cam)->flags & V4L2_BUF_FLAG_ERROR) || ret < 0)
				{
					if (retry < 100)
					{
						retry++;
						continue;
					}
					//					printf("ERROR in DQBUF\n");
					// stream_data.dq_err &= ~(1 << cam);
					// return 1;
				}
				else
				{
					usec[cam] = ((buffer + cam)->timestamp.tv_sec) * 1000000 + (buffer + cam)->timestamp.tv_usec;
					if (cam == 0)
					{
						high_usec = usec[cam];
						low_usec = usec[cam];
					}
					else if (usec[cam] > high_usec)
					{
						high_usec = usec[cam];
					}
					else if (usec[cam] < low_usec)
					{
						low_usec = usec[cam];
					}
					stream_data.dq_err |= (1 << cam);
				}
			}
		}
		// Timestamp based sync logic
		// 0x0f (0000 1111) represent Enable sync for 4 cameras
		if (frame_sync_data.sync_flag == stream_data.num_cam)
		{
			/* Software sync logic in case of lagging */
			for (cam = 0; cam < app_data->cameras_connected; cam++)
			{
				if ((stream_data.num_cam & (1 << cam)) && (stream_data.dq_err & (1 << cam)) && (high_usec - usec[cam]) > SYNC_TOLERANCE_US)
				{
					v4l2_capture_q_v4l2_buffer(app_data->camera_dev[cam], buffer + cam);
					v4l2_capture_dq_v4l2_buffer(app_data->camera_dev[cam], buffer + cam);
					usec[cam] = ((buffer + cam)->timestamp.tv_sec) * 1000000 + (buffer + cam)->timestamp.tv_usec;
					DEBUG(3, "Cam No. %d timestamp after DeQueue %lu\n", cam, usec[cam]);
				}
			}
			/*  All frame are in SYNC or not */
			for (cam = 0; cam < app_data->cameras_connected; cam++)
			{
				if ((stream_data.num_cam & (1 << cam)) && (stream_data.dq_err & (1 << cam)) && ((high_usec - usec[cam]) < SYNC_TOLERANCE_US))
					cam_sync |= (1 << cam);
			}
			// All three camera frames are in sync.. return these frames to display!
			if (frame_sync_data.sync_flag == cam_sync)
			{
				frame_sync_data.fsyn = FRAMES_IN_SYNC;
				return 0;
			}
			else
				cam_sync = 0; // reset the cam_sync count
		}
		else
		{
			frame_sync_data.fsyn = FRAMES_NOT_IN_SYNC;
			return 0; // FRAME_SYNC_CTRL is disabled
		}
		/* Queued all three camera frame if frame not in sync after Software logic */
		for (cam = 0; cam < app_data->cameras_connected; cam++)
		{
			v4l2_capture_q_v4l2_buffer(app_data->camera_dev[cam], buffer + cam);
		}
	}
	// Run the sync logic for MAX_SYNC_TRY_COUNT times and returning the non-sync buffers:
	frame_sync_data.fsyn = FRAMES_NOT_IN_SYNC;
	frame_sync_data.sync_flag = 0x00;
	printf(RED "Failed to synchronize after 20 Counts\n" RESET);
	return 0;
}

int Black_background(struct app_data *app_data)
{
	if (cmdline.no_display == CLEAR)
	{
		stream_data.framebuffer = (unsigned char *)malloc((sizeof(unsigned char)) * stream_data.display_size);
		if (!stream_data.framebuffer)
		{
			fprintf(stderr, "malloc: %s \n", strerror(errno));
			return -1;
		}
		// fill_display_background(stream_data.framebuffer, stream_data.display_size);
	}
	return 0;
}

void start_counting_frame(void)
{
	gettimeofday(&frame_rate_data.frame, NULL);
	if (frame_rate_data.flag == 0)
	{
		frame_rate_data.flag = 1;
		frame_rate_data.start = frame_rate_data.frame.tv_sec;
	}
	frame_rate_data.count_original++;
}

void compute_frame_rate(void)
{
	gettimeofday(&frame_rate_data.frame, NULL);
	frame_rate_data.end = frame_rate_data.frame.tv_sec;

	if (frame_rate_data.end > frame_rate_data.start)
	{
		frame_rate_data.flag = 0;
		if (frame_rate_data.show_frame_rate || (cmdline.no_display == SET))
		{
			fprintf(stderr,
					CLEAR_ENTIRE_LINE BLUE
					"\r\t\t\tvalid fps=%d original fps=%d" RESET,
					frame_rate_data.count_valid,
					frame_rate_data.count_original);
		}
		/* Frame count for video recording Once frame count done
		 * update record_count for stopping the application
		 */
		// record_data.frame_count++;
		// if(record_data.frame_count == cmdline.record_time + 1)
		// 	record_data.record_count = cmdline.record_time * DEFAULT_FRAMERATE_IN_SYNC;
		frame_rate_data.count_valid = 0;
		frame_rate_data.count_original = 0;
	}
}

void queued_all_camera_buffers(struct app_data *app_data, struct v4l2_buffer *camera_buffer)
{
	int cam = 0;
	for (cam = 0; cam < app_data->cameras_connected; cam++)
	{
		if ((stream_data.num_cam & (1 << cam)) && (stream_data.dq_err & (1 << cam)))
		{
			v4l2_capture_q_v4l2_buffer(app_data->camera_dev[cam], camera_buffer + cam);
		}
	}
}

void stop_v4l2_streaming(struct app_data *app_data)
{
	int cam = 0;
	for (cam = 0; cam < app_data->cameras_connected; cam++)
	{
		if (stream_data.num_cam & (1 << cam))
		{
			v4l2_capture_stop(app_data->camera_dev[cam]);
		}
	}
}
void free_application_data(struct app_data *app_data)
{
	int cam = 0, i;
	for (cam = 0; cam < app_data->cameras_connected; cam++)
	{
		for (i = 0; i < CAPTURE_MAX_BUFFER; i++)
		{
			if (app_data->camera_dev[cam] && app_data->camera_dev[cam]->buffer_pool[i].ptr)
			{
				free(app_data->camera_dev[cam]->buffer_pool[i].ptr);
				app_data->camera_dev[cam]->buffer_pool[i].ptr = NULL;
			}
		}
	}
}

void free_image_buffers(struct app_data *app_data)
{
	int cam = 0;
	for (cam = 0; cam < app_data->cameras_connected; cam++)
	{
		free_buffer(app_data->camera_dev[cam]);
		// free_buffer(st_image_data[cam].image_buf);
	}
}

// void free_gstreamer_buffers(struct app_data *app_data, struct gst_handler *gst_handle[] )
// {
// 	free_gst_handle(gst_handle[0]);

// 	if(cmdline.record != 1){
// 		free_buffer(stream_data.framebuffer);
// 		free_gst_handle(gst_handle[1]);
// 	}
// }
