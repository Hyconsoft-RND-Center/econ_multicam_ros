/*  tricam.elf Application
 *
 *  method used : user pointer allocation (seperate memory for each camera)
 *
 *  Image capture, with choice of displaying frames on screen given by user.
 */

#include "main.h"
#define SIGXERROR 10
struct sigaction sig_xerr;
uint16_t XERROR_VALUE = 100;
void sig_event_handler(int n, siginfo_t *info, void *unused);
#if 1

enum
{
	CLOCK_ERROR = 0,
	SEQUENCE_MONITOR_ERROR = 1,
	SYNC_MONITOR_ERROR = 2,
	DUAL_TEMP_ERROR1 = 3,
	DUAL_TEMP_ERROR2 = 4,
	DUAL_TEMP_ERROR3 = 5,
	SRAM_ECC_ERROR1 = 6,
	SRAM_ECC_ERROR2 = 7,
	SRAM_ECC_ERROR3 = 8,
	OTP_CRC_ERROR1 = 9,
	OTP_CRC_ERROR2 = 10,
	OTP_CRC_ERROR3 = 11,
	OTP_CRC_ERROR4 = 12,
	OTP_CRC_ERROR5 = 13,
	OTP_CRC_ERROR6 = 14,
	ROM_CRC_ERROR1 = 15,
	ROM_CRC_ERROR2 = 16,
	ROM_CRC_ERROR3 = 17,
	ABIST_CHECK_ERROR = 18,
	DATA_PATH_TEST_ERROR = 19,
	REGISTER_MONITOR_ERROR = 20,
	DUAL_LOCK_MONITOR_ERROR = 21,
	INTERNAL_BUS_ERROR = 23,
	ROW_COLUMN_ID_CHECK_ERROR = 24,
	POWER_MONITOR_ERROR1 = 25,
	POWER_MONITOR_ERROR2 = 26,
	POWER_MONITOR_ERROR3 = 27,
	WWDG_TIMER_ERROR = 28,
	FLASH_CHECK_ERROR1 = 29,
	FLASH_CHECK_ERROR2 = 30,
	FLASH_CHECK_ERROR3 = 31,
	CONTROL_FLOW_MONITOR_ERROR = 32,
	DMAC_MONITOR_ERROR = 33,
	POWER_SWITCH_MONITOR_ERROR = 34,
	CCC_PEELING_CHECK_ERROR = 35,
	MEMORY_PROTECTION_ERROR = 36,
	LBIST_CHECK_ERROR = 37,
	MBIST_CHECK_ERROR = 38,
	COMMUNICATION_CRC_ERROR = 39,
};

#endif

int ret_val = 0;

void print_help(char *argv[])
{
	fprintf(stdout, "Usage: multicam.elf [OPTION]... \n"
					" Arguments to the application: \n"
					" -w, --width=WIDTH       	width of the desired resolution \n"
					" -h, --height=HEIGHT     	height of the desired resolution \n"
					" -l, --list_res=[device] 	list the resolutions supported by the video node \n"
					" -n, --num_cam=NUM_CAM         stream only the first n number of cameras \n"
					" -d, --no-display=[1]    	set no-display streaming to 1 \n"
					" -s, --no-sync=[1]       	set no-sync to 1 for disable sync mode \n"
					" -r, --record=[1]        	set record to 1 for Video Recording \n"
					" -f, --record-format=[1/2] 	Available Video Recording format 1.H264 2.UYVY \n"
					"				Default Video Recording format:[H264]\n"
					" -t, --record-time=[time]	Video record time in seconds 	 \n"
					" 				Default Record time [H264=30-Sec] [UYVY=5-Sec] \n"
					" -v, --version           	prints the application version \n"
					" --help                  	prints the application usage \n"
					"Default resolution : %dx%d\n",
			DEFAULT__WIDTH,
			DEFAULT__HEIGHT);
	exit(1);
}

int parse_args(int argc, char *argv[])
{
	struct option long_options[] = {
		{"width", required_argument, 0, 'w'},
		{"height", required_argument, 0, 'h'},
		{"list_res", required_argument, 0, 'l'},
		{"num_cam", required_argument, 0, 'n'},
		{"no-display", required_argument, 0, 'd'},
		{"no-sync", required_argument, 0, 's'},
		{"record", required_argument, 0, 'r'},
		{"record-format", required_argument, 0, 'f'},
		{"record-time", required_argument, 0, 't'},
		{"version", no_argument, 0, 'v'},
		{"help", no_argument, 0, 'a'},
		{NULL, 0, NULL, 0}};
	int c;
	int def_w = 1;
	int def_h = 1;
	int option_index = 0;

	while ((c = getopt_long(argc, argv, "w:h:l:n:f:t:d:s:r:v", long_options, &option_index)))
	{
		if (c == -1)
			break;

		switch (c)
		{
		case 'w':
			def_w = 0;
			cmdline.width = atoi(optarg);
			cmdline.s_width = cmdline.width;
			break;
		case 'h':
			def_h = 0;
			cmdline.height = atoi(optarg);
			cmdline.s_height = cmdline.height;
			break;
		case 'l':
			list_resolution(optarg);
			device_data.exit_flag = 1;
			break;
		case 'f':
			cmdline.record_format = atoi(optarg);
			break;
		case 't':
			cmdline.record_time = atoi(optarg);
			break;
		case 'd':
			cmdline.no_display = atoi(optarg);
			if (cmdline.no_display != SET)
				cmdline.no_display = CLEAR;
			break;
		case 'r':
			cmdline.record = atoi(optarg);
			break;
		case 'v':
			// printf("\t\tVersion: %s\n", VERSION);
			device_data.exit_flag = 1;
			break;
		case 's':
			cmdline.disable_sync = atoi(optarg);
			if (cmdline.disable_sync == SET)
				frame_sync_data.sync_flag = 0x00;
			else
				frame_sync_data.sync_flag = 0x3f;
			break;
		case 'n':
			cmdline.num_cam = atoi(optarg);
			break;
		default:
			print_help(argv);
			break;
		}
	}
	if (cmdline.record == 1 && cmdline.record_format == 0)
		cmdline.record_format = 1;
	if (cmdline.record == 1 && cmdline.record_time == 0)
	{
		if (cmdline.record_format == 1)
			cmdline.record_time = 30;
		else
			cmdline.record_time = 5;
	}
	if (cmdline.num_cam == 0)
	{
		cmdline.num_cam = MAX_CAM;
	}
	if (cmdline.record == 1)
		printf(RED "################# Video Recording in Progress ... ####################\n" RESET);
	if (device_data.exit_flag != 1)
	{
		// Resize frames based on the size of the display.
		if (def_w != 0 || def_h != 0)
		{
			printf("SETTING DEFAULT RESOLUTION\n");
			cmdline.width = DEFAULT__WIDTH;
			cmdline.height = DEFAULT__HEIGHT;
			cmdline.s_width = DEFAULT__WIDTH;
			cmdline.s_height = DEFAULT__HEIGHT;
		}
		cmdline.stream = 1;
	}
	/*
	 * Synchronous streaming not possible for 13MP and 4K Cinema Resolution
	 * Application terminated when >4K resolution provided for streaming.
	 */
	if (cmdline.width > WIDTH_UHD || cmdline.height > HEIGHT_UHD)
	{
		printf("RESOLUTION NOT SUPPORTED\n");
		device_data.exit_flag = 1;
	}
	return 0;
}

void print_args()
{
	printf("\nArguments set:");
	printf("\n\twidth  : %d\n\theight : %d\n\n", cmdline.width, cmdline.height);
}

#if 0
void INThandler_xerror(int sig)
{



	switch(sig){

		case SIGUSR1:
			sigemptyset(&sig_xerr.sa_mask);
	    		sig_xerr.sa_flags = (SA_SIGINFO | SA_RESTART | SA_NODEFER);
			sig_xerr.sa_sigaction = sig_event_handler;
			sigaction(SIGUSR1, &sig_xerr, NULL);
			break;

	}


}
#endif

void INThandler(int sig)
{

	switch (sig)
	{

	case SIGINT:
		signal(sig, SIG_IGN);
		printf(RESET "\n\t Did you hit Ctrl-C ? \n");
		printf("\t ******* EXITING ******* \n");
		device_data.exit_flag = 1;
		break;
	}
}

void sig_event_handler(int n, siginfo_t *info, void *unused)
{

	uint8_t camera_connected = 0;
	if (n == SIGXERROR)
	{

		//	printf("VALUE RECEIVED FROM SIGNAL IS info->si_int = %d\n",info->si_int);
		XERROR_VALUE = info->si_int;

		camera_connected = ((XERROR_VALUE >> 8) & 0xFF);
		XERROR_VALUE = (XERROR_VALUE & 0xFF);

		switch (camera_connected)
		{

		case 1:
			printf("\nERROR IN CAMERA MODULE CONNECTED TO PORT 30-0043\n");
			break;

		case 2:
			printf("\nERROR IN CAMERA MODULE CONNECTED TO PORT 30-0044\n");
			break;

		case 3:
			printf("\nERROR IN CAMERA MODULE CONNECTED TO PORT 30-0045\n");
			break;

		case 4:
			printf("\nERROR IN CAMERA MODULE CONNECTED TO PORT 30-0046\n");
			break;

		case 5:
			printf("\nERROR IN CAMERA MODULE CONNECTED TO PORT 31-0043\n");
			break;

		case 6:
			printf("\nERROR IN CAMERA MODULE CONNECTED TO PORT 31-0044\n");
			break;

		case 7:
			printf("\nERROR IN CAMERA MODULE CONNECTED TO PORT 31-0045\n");
			break;

		case 8:
			printf("\nERROR IN CAMERA MODULE CONNECTED TO PORT 31-0046\n");
			break;
		}
		// printf ("Received signal from kernel : Value = %02d\n",XERROR_VALUE);
	}

	switch (XERROR_VALUE)
	{

	case CLOCK_ERROR:
		printf("FAULT IN INTERNAL SENSOR CLOCK\n\r");
		break;

	case SEQUENCE_MONITOR_ERROR:
		printf("FAULT IN SEQUENCE MONITOR\n\r");
		break;

	case SYNC_MONITOR_ERROR:
		printf("FAULT IN SYNC MONITOR\n\r");
		break;

	case DUAL_TEMP_ERROR1:
		printf("FAULT IN TEMP-1 SENSOR\n\r");
		break;

	case DUAL_TEMP_ERROR2:
		printf("FAULT IN TEMP-2 SENSOR\n\r");
		break;

	case DUAL_TEMP_ERROR3:
		printf("FAULT IN TEMP-3 SENSOR\n\r");
		break;

	case SRAM_ECC_ERROR1:
		printf("FAULT IN SRAM-1 ECC\n\r");
		break;

	case SRAM_ECC_ERROR2:
		printf("FAULT IN SRAM-2 ECC\n\r");
		break;

	case SRAM_ECC_ERROR3:
		printf("FAULT IN SRAM-3 ECC\n\r");
		break;

	case OTP_CRC_ERROR1:
		printf("FAULT IN OTP CRC-1\n\r");
		break;

	case OTP_CRC_ERROR2:
		printf("FAULT IN OTP CRC-2\n\r");
		break;

	case OTP_CRC_ERROR3:
		printf("FAULT IN OTP CRC-3\n\r");
		break;

	case OTP_CRC_ERROR4:
		printf("FAULT IN OTP CRC-4\n\r");
		break;

	case OTP_CRC_ERROR5:
		printf("FAULT IN OTP CRC-5\n\r");
		break;

	case OTP_CRC_ERROR6:
		printf("FAULT IN OTP CRC-6\n\r");
		break;

	case ROM_CRC_ERROR1:
		printf("FAULT IN ROM-1\n\r");
		break;

	case ROM_CRC_ERROR2:
		printf("FAULT IN ROM-2\n\r");
		break;

	case ROM_CRC_ERROR3:
		printf("FAULT IN ROM-3\n\r");
		break;

	case ABIST_CHECK_ERROR:
		printf("FAULT IN ABIST\n\r");
		break;

	case DATA_PATH_TEST_ERROR:
		printf("FAULT IN DATA PATH\n\r");
		break;

	case REGISTER_MONITOR_ERROR:
		printf("FAULT IN REGISTER MONITOR\n\r");
		break;

	case DUAL_LOCK_MONITOR_ERROR:
		printf("FAULT IN DUAL LOCK MODE\n\r");
		break;

	case INTERNAL_BUS_ERROR:
		printf("FAULT IN INTERNAL BUS\n\r");
		break;

	case ROW_COLUMN_ID_CHECK_ERROR:
		printf("FAULT IN ROW & COLUMN ID CHECK\n\r");
		break;

	case POWER_MONITOR_ERROR1:
		printf("FAULT IN POWER MONITOR-1\n\r");
		break;

	case POWER_MONITOR_ERROR2:
		printf("FAULT IN POWER MONITOR-2\n\r");
		break;

	case POWER_MONITOR_ERROR3:
		printf("FAULT IN POWER MONITOR-3\n\r");
		break;

	case WWDG_TIMER_ERROR:
		printf("FAULT IN WATCH DOG\n\r");
		break;

	case FLASH_CHECK_ERROR1:
		printf("FAULT IN FLASH CHECK-1\n\r");
		break;

	case FLASH_CHECK_ERROR2:
		printf("FAULT IN FLASH CHECK-2\n\r");
		break;

	case FLASH_CHECK_ERROR3:
		printf("FAULT IN FLASH CHECK-3\n\r");
		break;

	case CONTROL_FLOW_MONITOR_ERROR:
		printf("FAULT IN CONTROL FLOW MONITOR\n\r");
		break;

	case DMAC_MONITOR_ERROR:
		printf("FAULT IN DMAC MONITOR\n\r");
		break;

	case POWER_SWITCH_MONITOR_ERROR:
		printf("FAULT IN POWER SWITCH MONITOR\n\r");
		break;

	case CCC_PEELING_CHECK_ERROR:
		printf("FAULT IN CCC PEELING CHECK \n\r");
		break;

	case MEMORY_PROTECTION_ERROR:
		printf("FAULT IN MEMORY PROTECTION\n\r");
		break;

	case LBIST_CHECK_ERROR:
		printf("FAULT IN LBIST\n\r");
		break;

	case MBIST_CHECK_ERROR:
		printf("FAULT IN MBIST CHECK ERROR\n\r");
		break;

	case COMMUNICATION_CRC_ERROR:
		printf("FAULT IN COMMUNICATION CRC ERROR\n\r");
		break;
	}
	XERROR_VALUE = 100;
}

int main(int argc, char *argv[])
{
	struct app_data *app_data = NULL;
	unsigned char *fullbuffer = NULL;
	/* 2 handles : one is for stream/record and
	 * another is for background fill in stream mode
	 */
	// struct gst_handler *gst_handle[2]={NULL};
	short int cam = 0, i;

	//	pthread_t control_tid;
	//	int active = 1, err = 0;
	struct v4l2_buffer camera_buffer[MAX_CAM]; // v4l2 camera buffers
	guint8 *cap_ptr[MAX_CAM];
	
	start_AA();
	wait_for_AA_running();

	init_app_data();

	sigemptyset(&sig_xerr.sa_mask);
	sig_xerr.sa_flags = (SA_SIGINFO | SA_RESTART | SA_NODEFER);
	sig_xerr.sa_sigaction = sig_event_handler;
	sigaction(SIGUSR1, &sig_xerr, NULL);
	//	signal(SIGUSR1, INThandler_xerror);

	parse_args(argc, argv);
	if (device_data.exit_flag)
		return 0;
	print_args();

	/* Identify the Jetson Device  TX1/TX2/XAVIER */
	device_data.chip_id = tegra_get_chip_id();
	if (device_data.chip_id < 0)
	{
		fprintf(stderr, "Unable to read CHIP ID \n");
		return -1;
	}

	app_data = (struct app_data *)malloc(sizeof(struct app_data));
	if (app_data == NULL)
	{
		fprintf(stderr, "appdata allocation failed\n");
		return -1;
	}

	if (init_buffer_size(cmdline))
	{
		fprintf(stderr, "Failed to init_buffer_size\n");
		return -1;
	}

	/* Dynamically finding the total number of cameras connected with Jeston Camera Connector */
	get_total_camera_connected(app_data);
	if (app_data->cameras_connected == 0)
	{
		printf("\n\nNO CAMERA NODES PRESENT\n\n");
		free(app_data);
		app_data = NULL;
		return 0;
	}

	if (cmdline.num_cam < app_data->cameras_connected)
	{
		app_data->cameras_connected = cmdline.num_cam;
	}

	if (init_v4l2_camera(app_data, &cmdline, camera_buffer))
	{
		fprintf(stderr, "Failed to init_v4l2_camera \n");
		free(app_data);
		return -1;
	}

	/* Updated the SYNC FLAG Value Based on How many camera connected
	 * if Frame sync enable
	 * For 1 CAM sync_flag = 0x01 (0000 0001)
	 * For 2 CAM sync_flag = 0x03 (0000 0011)
	 * For 3 CAM sync_flag = 0x07 (0000 0111)
	 * For 4 CAM sync_flag = 0x0f (0000 1111)
	 * For 5 CAM sync_flag = 0x1f (0001 1111)
	 * For 6 CAM sync_flag = 0x3f (0011 1111)
	 */
	if (frame_sync_data.sync_flag == 0xff)
		frame_sync_data.sync_flag = stream_data.num_cam;

	// if (init_gstreamer_handler( app_data, &cmdline, gst_handle)){
	// 	fprintf(stderr, "Failed to init_gstreamer_handler \n");
	// 	free(app_data);
	// 	return -1;
	// }

	/* Create controls thread */
	key_event_data.err = pthread_create(&key_event_data.control_tid, NULL, &init_control, (void *)app_data);
	if (key_event_data.err != 0)
	{
		printf("\n ERROR: Unable to create THREAD\n");
		// exit(EXIT_FAILURE);
	}

	/* Allocate memory for  BackGround Filling in case of 2 ,3 ,5 & 6 cameras connected to the system. */
	if (Black_background(app_data))
	{
		fprintf(stderr, "Failed to fill Background \n");
	}

	for (i = 0; i < CAPTURE_MAX_BUFFER; i++)
	{
		for (cam = 0; cam < app_data->cameras_connected; cam++)
		{
			if ((stream_data.num_cam & (1 << cam)) && (stream_data.dq_err & (1 << cam)))
			{
				v4l2_capture_dq_v4l2_buffer(app_data->camera_dev[cam], camera_buffer + cam);
				v4l2_capture_q_v4l2_buffer(app_data->camera_dev[cam], camera_buffer + cam);
			}
		}
	}

	/* Allocate memory for Image capture buffer
	 * Memory allocation =
	 * 	(Total no. of cameras connected * save_image_len)
	 * 	Only In case of 5 camera one extra image size
	 * 	memory allocation required for image capture
	 */
	if (app_data->cameras_connected == 5)
	{
		fullbuffer = (unsigned char *)malloc((sizeof(unsigned char)) *
											 (img_common_data.save_image_size * (app_data->cameras_connected + 1)));
	}
	else
	{
		fullbuffer = (unsigned char *)malloc((sizeof(unsigned char)) *
											 (img_common_data.save_image_size * (app_data->cameras_connected)));
	}
	if (!fullbuffer)
	{
		fprintf(stderr, "malloc: %s \n", strerror(errno));
		exit(-errno);
	}

	for (int cam = 0; cam < app_data->cameras_connected; cam++)
	{
		char node_name[50], topic_name[50];
		snprintf(node_name, sizeof(node_name), "video_publisher_%d", cam);
		snprintf(topic_name, sizeof(topic_name), "/dev/video%d", cam);
		app_data->publisher_ctx[cam] = create_publisher(argc, (const char *const *)argv, node_name, topic_name);
		app_data->publisher_ctx[cam]->image_msg = create_message_struct(cmdline.height, cmdline.width);

		if (app_data->publisher_ctx[cam] == NULL)
		{
			fprintf(stderr, "Failed to create publisher for camera %d\n", cam);
			// Handle the error, e.g., clean up other publishers or resources if necessary
		}
	}

	while (key_event_data.active)
	{
		if (!device_data.exit_flag)
		{

			start_counting_frame();

			/* Get timestamp based sync frames */
			ret_val = get_camera_frames(app_data, camera_buffer);
			if (ret_val == 1)
			{
				device_data.exit_flag = 1;
				printf("device_data.exit_flag = 1 is set");
			}
			signal(SIGINT, INThandler);

			for (cam = 0; cam < app_data->cameras_connected; cam++)
			{
				if ((stream_data.dq_err & (1 << cam)))
				{
					// Capture the frame
					cap_ptr[cam] = get_frame_virt(app_data, camera_buffer + cam, cam);

					if (cap_ptr[cam] != NULL)
					{
						// set_current_time(&image_msg.header.stamp);
						set_current_time(&app_data->publisher_ctx[cam]->image_msg->header.stamp);
						memcpy(app_data->publisher_ctx[cam]->image_msg->data.data, cap_ptr[cam], app_data->publisher_ctx[cam]->image_msg->data.size);
						rcl_ret_t rc;
						rc = rcl_publish(&app_data->publisher_ctx[cam]->publisher, app_data->publisher_ctx[cam]->image_msg, NULL);
						if (rc != RCL_RET_OK)
						{
							printf("Error publishing message: %s\n", rcl_get_error_string().str);
						}
						memset(app_data->publisher_ctx[cam]->image_msg->data.data, 0, app_data->publisher_ctx[cam]->image_msg->data.size);
					}
					else
					{
						printf("\n INFO: cap_ptr[cam] is NULL \n");
					}
				}
			}

			// if (cmdline.no_display == SET)
			// 	frame_rate_data.count_valid++;
			// else if (cmdline.record != 1) {
			// 	preview_frames_on_display(cap_ptr, gst_handle, stream_data.framebuffer,
			// 			app_data->cameras_connected, img_common_data.buffer_size);
			// } else if (cmdline.record == 1 && record_data.frame_count != 0) {
			// 	video_record(cap_ptr, gst_handle[0], app_data->cameras_connected, img_common_data.buffer_size);
			// 	frame_rate_data.count_valid++;
			// }

			/*
			 * Application terminate If recording done and exit_flag set once recording done
			 * For example:-  Record Time == 5 sec then total capture frame 150
			 */
			// if ((record_data.record_count == (cmdline.record_time * DEFAULT_FRAMERATE_IN_SYNC)) && cmdline.record == 1) {
			// 	record_data.record_count=0;
			// 	device_data.exit_flag =1;
			// }

			compute_frame_rate();

			/* Image Capture based on user input option */
			// if (img_common_data.capture_mode != CAPT_MODE_DISABLE || img_common_data.writing_flag != CLEAR)
			// image_capture(stream_data.num_cam, cap_ptr, fullbuffer, img_common_data.save_image_size, cmdline,app_data);

			/* QUEUE all connected  camera buffers */
			queued_all_camera_buffers(app_data, camera_buffer);
		}
		else
		{
			/* Stop streaming and free buffers */

			stop_v4l2_streaming(app_data);

			free_application_data(app_data);

			free_image_buffers(app_data);

			free_buffer(fullbuffer);

			for (cam = 0; cam < app_data->cameras_connected; cam++)
			{
				rcl_ret_t rc;
				rc = rcl_node_fini(&app_data->publisher_ctx[cam]->node);
				if (rc != RCL_RET_OK) 
				{
    				fprintf(stderr, "Error: Function returned %d\n", rc);
				}
				free(app_data->publisher_ctx[cam]);
			}

			// free_gstreamer_buffers(app_data, gst_handle);

			free_buffer(app_data);
			return 0;
		}
	}
	return 0;
}

void free_buffer(void *buffer)
{
	if (buffer != NULL)
	{
		free(buffer);
		buffer = NULL;
	}
}

/**
 * @brief : get capture buffer virtual address
 *
 * @param capture_buff: v4l2 capture buffer
 *
 * @return : virtual address of the captured frame
 */
guint8 *get_frame_virt(struct app_data *appdata,
					   struct v4l2_buffer *capture_buff, int camera_num)
{
	return (guint8 *)appdata->camera_dev[camera_num]->buffer_pool[capture_buff->index].ptr;
}
