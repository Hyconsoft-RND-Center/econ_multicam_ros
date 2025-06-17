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

    printf("Publisher initialized successfully: %s\n", topic_name);
    return pub_ctx;
}

sensor_msgs__msg__Image *create_message_struct(int height, int width)
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
    const char *frame_id = "camera";
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

    // Set the encoding
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
    image_msg->step = width * 2; // 2 bytes per pixel for YUYV

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
