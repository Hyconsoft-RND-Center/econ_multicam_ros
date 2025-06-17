#include <stdio.h>
#include <string.h>
#include <rcl/time.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/image.h>
#include <rcl/time.h>
#include <builtin_interfaces/msg/time.h>

typedef struct
{
    rcl_node_t node;
    rcl_publisher_t publisher;
    sensor_msgs__msg__Image *image_msg;
} PublisherContext;

PublisherContext *create_publisher(int argc, const char *const argv[], const char *node_name, const char *topic_name);
sensor_msgs__msg__Image *create_message_struct(int height, int width);
void set_current_time(builtin_interfaces__msg__Time *stamp);
