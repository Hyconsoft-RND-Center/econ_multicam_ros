#include "main.h"
/* Geting Tegra Processor CHIP ID
	for TX1 	= 33
	for TX2 	= 24
	for XAVIER 	= 25
*/
int tegra_get_chip_id(void)
{
#if 0
	int fd;
	char *id=malloc(sizeof(char) * 4096);
	fd= open("/sys/module/tegra_fuse/parameters/tegra_chip_id",O_RDONLY);
	if(fd < 0) {
		printf("File not open\n");
		return -1;
	}
	/* Reading chip id */
	read(fd, id, 4096);
	close(fd);
	free(id);	
	
	return atoi(id);
#endif
	return 0;
}

int get_total_camera_connected(struct app_data *app_data)
{
	char *dev_node = (char *)malloc((sizeof(char)) * 16);
	int fd;
	struct v4l2_capability capability;
	int cam = 0;

	for (cam = 0; cam < MAX_CAM; cam++)
	{
		sprintf(dev_node, DEV_NODE, cam);

		fd = open(dev_node, O_RDWR);
		if (fd < 0)
		{
			free(dev_node);
			if (cam == 0)
				app_data->cameras_connected = cam;
			return 0;
		}
		if (ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0)
		{
			perror("Error: VIDIOC_QUERYCAP: ");
			return -1;
		}
		if (strcmp((const char *)capability.driver, "tegra-video") == 0)
			app_data->cameras_connected = cam + 1;
		close(fd);
	}
	free(dev_node);
	return app_data->cameras_connected;
}
