#include "ros_publisher.h"

PublisherContext *create_publisher(int argc, const char *const argv[], const char *node_name, const char *topic_name)
{
    rcl_ret_t ret;

    PublisherContext *pub_ctx = (PublisherContext *)malloc(sizeof(PublisherContext));
    if (pub_ctx == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for publisher context\n");
        return NULL;
    }

    rclc_support_t support;
    rcl_allocator_t allocator = rcl_get_default_allocator();
    ret = rclc_support_init(&support, argc, argv, &allocator);
    if (ret != RCL_RET_OK)
    {
        fprintf(stderr, "Failed to initialize support: %s\n", rcl_get_error_string().str);
        free(pub_ctx);
        return NULL;
    }

    // Initialize the node
    ret = rclc_node_init_default(&pub_ctx->node, node_name, "", &support);
    if (ret != RCL_RET_OK)
    {
        fprintf(stderr, "Failed to initialize node: %s\n", rcl_get_error_string().str);
        rclc_support_fini(&support);
        free(pub_ctx);
        return NULL;
    }

    const rosidl_message_type_support_t *msg_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image);
    ret = rclc_publisher_init_default(&pub_ctx->publisher, &pub_ctx->node, msg_type_support, topic_name);
    if (ret != RCL_RET_OK)
    {
        fprintf(stderr, "Failed to initialize publisher for topic: %s - %s\n", topic_name, rcl_get_error_string().str);
        if ((rcl_node_fini(&pub_ctx->node)) != RCL_RET_OK) 
		{
    	    fprintf(stderr, "Error: Failed to finalize publisher \n");
		}
        rclc_support_fini(&support);
        free(pub_ctx);
        return NULL;
    }

    // Initialize camera_info publisher
    char camera_info_topic[256];
    snprintf(camera_info_topic, sizeof(camera_info_topic), "%s/camera_info", topic_name);
    const rosidl_message_type_support_t *camera_info_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CameraInfo);
    ret = rclc_publisher_init_default(&pub_ctx->camera_info_publisher, &pub_ctx->node, camera_info_type_support, camera_info_topic);
    if (ret != RCL_RET_OK)
    {
        fprintf(stderr, "Failed to initialize camera_info publisher for topic: %s - %s\n", camera_info_topic, rcl_get_error_string().str);
        rcl_publisher_fini(&pub_ctx->publisher, &pub_ctx->node);
        if ((rcl_node_fini(&pub_ctx->node)) != RCL_RET_OK) 
		{
    	    fprintf(stderr, "Error: Failed to finalize publisher \n");
		}
        rclc_support_fini(&support);
        free(pub_ctx);
        return NULL;
    }

    printf("Publisher initialized successfully: %s\n", topic_name);
    printf("Camera info publisher initialized successfully: %s\n", camera_info_topic);
    return pub_ctx;
}

sensor_msgs__msg__Image *create_message_struct(int height, int width, const char *frame_id)
{
    sensor_msgs__msg__Image *image_msg = (sensor_msgs__msg__Image *)malloc(sizeof(sensor_msgs__msg__Image));
    if (image_msg == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for image message\n");
        return NULL;
    }

    // Initialize the message
    if (!sensor_msgs__msg__Image__init(image_msg))
    {
        fprintf(stderr, "Failed to initialize image message\n");
        free(image_msg);
        return NULL;
    }

    // Set the header frame ID
    size_t frame_id_length = strlen(frame_id);
    image_msg->header.frame_id.data = (char *)malloc(frame_id_length + 1);
    if (image_msg->header.frame_id.data == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for frame ID\n");
        sensor_msgs__msg__Image__fini(image_msg);
        free(image_msg);
        return NULL;
    }
    strcpy(image_msg->header.frame_id.data, frame_id);
    image_msg->header.frame_id.size = frame_id_length;
    image_msg->header.frame_id.capacity = frame_id_length + 1;

    // Set the image dimensions
    image_msg->height = height;
    image_msg->width = width;

    // Set the encoding to YUV422 (2 bytes per pixel)
    const char *encoding = "yuv422";
    size_t encoding_length = strlen(encoding);
    image_msg->encoding.data = (char *)malloc(encoding_length + 1);
    if (image_msg->encoding.data == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for encoding\n");
        sensor_msgs__msg__Image__fini(image_msg);
        free(image_msg);
        return NULL;
    }
    strcpy(image_msg->encoding.data, encoding);
    image_msg->encoding.size = encoding_length;
    image_msg->encoding.capacity = encoding_length + 1;

    image_msg->is_bigendian = 0;
    image_msg->step = width * 2; // 2 bytes per pixel for YUV422

    // Allocate memory for the image data
    image_msg->data.data = (uint8_t *)malloc(image_msg->height * image_msg->step);
    if (image_msg->data.data == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for image data\n");
        sensor_msgs__msg__Image__fini(image_msg);
        free(image_msg);
        return NULL;
    }
    image_msg->data.size = image_msg->height * image_msg->step;
    image_msg->data.capacity = image_msg->height * image_msg->step;

    return image_msg;
}

void set_current_time(builtin_interfaces__msg__Time *stamp)
{
    rcl_time_point_value_t now;
    rcl_ret_t rc = rcutils_system_time_now(&now); // Using rcutils_system_time_now instead
    if (rc == RCL_RET_OK)
    {
        stamp->sec = (int32_t)(now / 1000000000);
        stamp->nanosec = (uint32_t)(now % 1000000000);
    }
    else
    {
        printf("Error getting current time: %s\n", rcl_get_error_string().str);
    }
}

sensor_msgs__msg__CameraInfo *create_camera_info_message(int height, int width, const char *frame_id, int device_id)
{
    sensor_msgs__msg__CameraInfo *camera_info_msg = (sensor_msgs__msg__CameraInfo *)malloc(sizeof(sensor_msgs__msg__CameraInfo));
    if (camera_info_msg == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for camera info message\n");
        return NULL;
    }

    // Initialize the message
    if (!sensor_msgs__msg__CameraInfo__init(camera_info_msg))
    {
        fprintf(stderr, "Failed to initialize camera info message\n");
        free(camera_info_msg);
        return NULL;
    }

    // Set the header frame ID
    size_t frame_id_length = strlen(frame_id);
    camera_info_msg->header.frame_id.data = (char *)malloc(frame_id_length + 1);
    if (camera_info_msg->header.frame_id.data == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for frame ID\n");
        sensor_msgs__msg__CameraInfo__fini(camera_info_msg);
        free(camera_info_msg);
        return NULL;
    }
    strcpy(camera_info_msg->header.frame_id.data, frame_id);
    camera_info_msg->header.frame_id.size = frame_id_length;
    camera_info_msg->header.frame_id.capacity = frame_id_length + 1;

    // Set camera parameters
    camera_info_msg->height = height;
    camera_info_msg->width = width;

    // Set distortion model
    const char *distortion_model = "plumb_bob";
    size_t model_length = strlen(distortion_model);
    camera_info_msg->distortion_model.data = (char *)malloc(model_length + 1);
    if (camera_info_msg->distortion_model.data == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for distortion model\n");
        sensor_msgs__msg__CameraInfo__fini(camera_info_msg);
        free(camera_info_msg);
        return NULL;
    }
    strcpy(camera_info_msg->distortion_model.data, distortion_model);
    camera_info_msg->distortion_model.size = model_length;
    camera_info_msg->distortion_model.capacity = model_length + 1;

    // Initialize distortion parameters (D) - 5 parameters for plumb_bob
    camera_info_msg->d.size = 5;
    camera_info_msg->d.capacity = 5;
    camera_info_msg->d.data = (double *)malloc(5 * sizeof(double));
    if (camera_info_msg->d.data == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for distortion parameters\n");
        sensor_msgs__msg__CameraInfo__fini(camera_info_msg);
        free(camera_info_msg);
        return NULL;
    }
    // Set calibration parameters based on device_id
    double fx, fy, cx, cy;
    double d[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // Default: no distortion
    
    switch(device_id) {
        case 0: // dev0 - front camera
            fx = 1730.96984; fy = 1744.46222; cx = 964.66126; cy = 465.84157;
            d[0] = -0.364515; d[1] = 0.130944; d[2] = 0.002280; d[3] = -0.002711; d[4] = 0.000000;
            break;
        case 1: // dev1 - back camera  
            fx = 1510.90848; fy = 1522.93426; cx = 930.1433; cy = 555.95368;
            d[0] = -0.308347; d[1] = 0.059721; d[2] = -0.002041; d[3] = 0.008979; d[4] = 0.000000;
            break;
        case 2: // dev2 - left camera
            fx = 907.59532; fy = 915.99537; cx = 951.22199; cy = 543.44269;
            d[0] = -0.295144; d[1] = 0.054535; d[2] = 0.000679; d[3] = -0.002782; d[4] = 0.000000;
            break;
        case 3: // dev3 - right camera
            fx = 772.44465; fy = 777.32646; cx = 979.71164; cy = 554.33627;
            d[0] = -0.243915; d[1] = 0.035940; d[2] = -0.005044; d[3] = -0.000517; d[4] = 0.000000;
            break;
        default: // Default values
            fx = width / 2.0; fy = height / 2.0; cx = width / 2.0; cy = height / 2.0;
            break;
    }
    
    // Set distortion coefficients
    for (int i = 0; i < 5; i++)
    {
        camera_info_msg->d.data[i] = d[i];
    }

    camera_info_msg->k[0] = fx; camera_info_msg->k[1] = 0.0; camera_info_msg->k[2] = cx;
    camera_info_msg->k[3] = 0.0; camera_info_msg->k[4] = fy; camera_info_msg->k[5] = cy;
    camera_info_msg->k[6] = 0.0; camera_info_msg->k[7] = 0.0; camera_info_msg->k[8] = 1.0;

    // Initialize rectification matrix (R) - 3x3 identity matrix
    camera_info_msg->r[0] = 1.0; camera_info_msg->r[1] = 0.0; camera_info_msg->r[2] = 0.0;
    camera_info_msg->r[3] = 0.0; camera_info_msg->r[4] = 1.0; camera_info_msg->r[5] = 0.0;
    camera_info_msg->r[6] = 0.0; camera_info_msg->r[7] = 0.0; camera_info_msg->r[8] = 1.0;

    // Initialize projection matrix (P) based on device_id
    switch(device_id) {
        case 0: // dev0 - front camera
            camera_info_msg->p[0] = 1512.38489; camera_info_msg->p[1] = 0.0; camera_info_msg->p[2] = 958.18693; camera_info_msg->p[3] = 0.0;
            camera_info_msg->p[4] = 0.0; camera_info_msg->p[5] = 1677.03149; camera_info_msg->p[6] = 461.05392; camera_info_msg->p[7] = 0.0;
            camera_info_msg->p[8] = 0.0; camera_info_msg->p[9] = 0.0; camera_info_msg->p[10] = 1.0; camera_info_msg->p[11] = 0.0;
            break;
        case 1: // dev1 - back camera
            camera_info_msg->p[0] = 1277.13574; camera_info_msg->p[1] = 0.0; camera_info_msg->p[2] = 949.04199; camera_info_msg->p[3] = 0.0;
            camera_info_msg->p[4] = 0.0; camera_info_msg->p[5] = 1459.5304; camera_info_msg->p[6] = 555.49523; camera_info_msg->p[7] = 0.0;
            camera_info_msg->p[8] = 0.0; camera_info_msg->p[9] = 0.0; camera_info_msg->p[10] = 1.0; camera_info_msg->p[11] = 0.0;
            break;
        case 2: // dev2 - left camera
            camera_info_msg->p[0] = 621.44672; camera_info_msg->p[1] = 0.0; camera_info_msg->p[2] = 959.98432; camera_info_msg->p[3] = 0.0;
            camera_info_msg->p[4] = 0.0; camera_info_msg->p[5] = 803.41724; camera_info_msg->p[6] = 545.21623; camera_info_msg->p[7] = 0.0;
            camera_info_msg->p[8] = 0.0; camera_info_msg->p[9] = 0.0; camera_info_msg->p[10] = 1.0; camera_info_msg->p[11] = 0.0;
            break;
        case 3: // dev3 - right camera
            camera_info_msg->p[0] = 552.39667; camera_info_msg->p[1] = 0.0; camera_info_msg->p[2] = 977.22865; camera_info_msg->p[3] = 0.0;
            camera_info_msg->p[4] = 0.0; camera_info_msg->p[5] = 664.03961; camera_info_msg->p[6] = 548.97974; camera_info_msg->p[7] = 0.0;
            camera_info_msg->p[8] = 0.0; camera_info_msg->p[9] = 0.0; camera_info_msg->p[10] = 1.0; camera_info_msg->p[11] = 0.0;
            break;
        default: // Default projection matrix
            camera_info_msg->p[0] = fx; camera_info_msg->p[1] = 0.0; camera_info_msg->p[2] = cx; camera_info_msg->p[3] = 0.0;
            camera_info_msg->p[4] = 0.0; camera_info_msg->p[5] = fy; camera_info_msg->p[6] = cy; camera_info_msg->p[7] = 0.0;
            camera_info_msg->p[8] = 0.0; camera_info_msg->p[9] = 0.0; camera_info_msg->p[10] = 1.0; camera_info_msg->p[11] = 0.0;
            break;
    }

    // Set binning parameters
    camera_info_msg->binning_x = 0;
    camera_info_msg->binning_y = 0;

    // Set ROI parameters (full image)
    camera_info_msg->roi.x_offset = 0;
    camera_info_msg->roi.y_offset = 0;
    camera_info_msg->roi.height = 0;
    camera_info_msg->roi.width = 0;
    camera_info_msg->roi.do_rectify = false;

    return camera_info_msg;
}
