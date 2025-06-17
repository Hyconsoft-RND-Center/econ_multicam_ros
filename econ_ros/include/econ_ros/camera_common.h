#define MAX_WIDTH 2304
#define MAX_HEIGHT 1536

struct cmdline_args
{
        int width;
        int height;
        int stream;
        int s_width;
        int s_height;
        int d_width;
        int d_height;
        int no_display;
        int disable_sync;
        int record;
        int record_format;
        int record_time;
        int num_cam;
};

struct image_common_data
{
        int writing_flag;
        unsigned int save_image_size;
        guint buffer_size;
        int image_capture_flag;
        int img_format;
        char capture_mode;
} img_common_data;
struct frame_rate_calculate
{
        int flag;
        int start;
        int end;
        int count_original;
        int count_valid;
        int show_frame_rate;
        struct timeval frame;
} frame_rate_data;

struct frame_sync
{
        int sync_flag;
        int fsyn;
        int g_skip;
} frame_sync_data;

void *init_control(void *data);
void init_app_data(void);
int get_buffer_size(struct cmdline_args cmdline);
int init_buffer_size(struct cmdline_args cmdline);
int v4l2_start_stream(struct app_data *app_data, struct cmdline_args *cmdline);
int init_v4l2_camera(struct app_data *app_data, struct cmdline_args *cmdline, struct v4l2_buffer *camera_buffer);
// int init_gstreamer_handler( struct app_data *app_data, struct cmdline_args *cmdline, struct gst_handler **gst_handle);
int get_camera_frames(struct app_data *app_data, struct v4l2_buffer *buffer);
int Black_background(struct app_data *app_data);
void start_counting_frame(void);
void compute_frame_rate(void);
void queued_all_camera_buffers(struct app_data *app_data, struct v4l2_buffer *camera_buffer);
void stop_v4l2_streaming(struct app_data *app_data);
void free_application_data(struct app_data *app_data);
void free_image_buffers(struct app_data *app_data);
// void free_gstreamer_buffers(struct app_data *app_data, struct gst_handler *gst_handle[] );