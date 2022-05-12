/******************************************************************************
*
*   Copyright (c) 2020 Intel.
*
*   Licensed under the Apache License, Version 2.0 (the "License");
*   you may not use this file except in compliance with the License.
*   You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*
*******************************************************************************/

#include <stdlib.h>
#include <stdint.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <strings.h>
#include <unistd.h>
#include <stdbool.h>
#include <limits.h>
#include <linux/vfio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <libgen.h>

#include "acc100_cfg_app.h"
#include "fpga_lte_cfg_app.h"
#include "fpga_5gnr_cfg_app.h"

#define SYS_DIR "/sys/bus/pci/devices"
#define CUR_DIR "."
#define PREV_DIR ".."

#define DEVICE_FILE  "device"
#define VENDOR_FILE  "vendor"
#define BAR0_FILE    "resource0"

#define PCI_STR_SIZE 15
#define NULL_PAD     2

#define ACC100_VENDOR_ID        0x8086
#define ACC100_DEVICE_ID        0x0D5C
#define FPGA_LTE_FEC_VENDOR_ID  0x1172
#define FPGA_LTE_FEC_DEVICE_ID  0x5052
#define FPGA_5GNR_FEC_VENDOR_ID 0x8086
#define FPGA_5GNR_FEC_DEVICE_ID 0x0D8F

#define VFIO_VF_TOKEN_LEN 16
#define VFIO_VF_TOKEN_STR_LEN 36
#define SLEEP_30SEC 30

/* Function Pointer for device specific configuration file */
typedef int (*configuration)(void *bar0addr, const char *arg_cfg_filename);

typedef struct hw_device {
	const char *device_name;
	char *config_file;
	int vendor_id;
	int device_id;
	char pci_address[PCI_STR_SIZE];
	bool driver_found;
	configuration conf;
	int config_all;
	int vfio_mode;
	unsigned char vfio_vf_token[VFIO_VF_TOKEN_LEN];
} hw_device;


static int vfio_get_device_groupid(const char *pci_addr)
{
	char device_iommu_group[PATH_MAX];
	char group_path[PATH_MAX];
	char *group_name;
	int groupid = -1;
	int len;

	snprintf(device_iommu_group, sizeof(device_iommu_group),
			"%s/%s/iommu_group", SYS_DIR, pci_addr);
	len = readlink(device_iommu_group, group_path, sizeof(group_path));
	if (len < 0) {
		printf("ERR:VFIO: iommu_group error for %s\n", pci_addr);
		return -1;
	}
	group_path[len] = 0;

	group_name = basename(group_path);
	groupid = strtol(group_name, NULL, 10);
	if (groupid == 0) {
		printf("ERR:VFIO: Faild to read %s\n", group_path);
		return -1;
	}

	return groupid;
}

static int uuid_parse(char *uuid_str, unsigned char *uuid)
{
	int i, inx;
	int high_n = 1;
	unsigned char tmp;

	if (strlen(uuid_str) != VFIO_VF_TOKEN_STR_LEN) {
		printf("ERR:uuid string len is wrong: %d\n", (int)strlen(uuid_str));
		return -1;
	}
	for (i = 0, inx = 0;
		((i < VFIO_VF_TOKEN_STR_LEN) && (inx < VFIO_VF_TOKEN_LEN)); i++) {
		if (uuid_str[i] == '-') {
			if ((i ==  8) || (i == 13) || (i == 18) || (i == 23))
				continue;
		}
		if (!isxdigit(uuid_str[i])) {
			printf("ERR: unknown char in uuid string\n");
			return -1;
		}
		tmp = isdigit(uuid_str[i]) ? uuid_str[i] - '0' :
				tolower(uuid_str[i]) - 'a' + 0xA;

		if (high_n) {
			uuid[inx] = (tmp & 0xF) << 4;
			high_n = 0;
		} else {
			uuid[inx++] |= (tmp & 0xF);
			high_n = 1;
		}
	}

	return 0;
}

#ifdef VFIO_DEVICE_FEATURE
static void uuid_unparse(unsigned char *uuid, char *uuid_str)
{
	char *ptr = uuid_str;
	int temp;
	int i;

	for (i = 0; i < VFIO_VF_TOKEN_LEN; i++) {
		if (i == 4 || i == 6 || i == 8 || i == 10)
			*ptr++ = '-';

		temp = (uuid[i] >> 4) & 0xF;
		*ptr++ = (temp < 10) ? temp + '0' : 'a' + (temp - 10);
		temp = (uuid[i] & 0xF);
		*ptr++ = (temp < 10) ? temp + '0' : 'a' + (temp - 10);
	}
	*ptr = '\0';
}
#endif

static int vfio_set_token(hw_device *hwd, int vfio_dev_fd)
{
#ifdef VFIO_DEVICE_FEATURE
	int ret;
	char uuid_string[VFIO_VF_TOKEN_STR_LEN+1];
	struct vfio_device_feature *device_feature;

	device_feature = (struct vfio_device_feature *)
			malloc(sizeof(struct vfio_device_feature) + VFIO_VF_TOKEN_LEN);
	if (device_feature == NULL) {
		printf("ERR:VFIO: memory allocaton failed\n");
		return -1;
	}

	/* Set the secret token shared between PF and VF */
	printf("INFO:VFIO: Setting VFIO_DEVICE_FEATURE with UUID token : ");

	device_feature->argsz = sizeof(device_feature) + VFIO_VF_TOKEN_LEN;
	device_feature->flags = VFIO_DEVICE_FEATURE_SET | VFIO_DEVICE_FEATURE_PCI_VF_TOKEN;
	memcpy(device_feature->data, hwd->vfio_vf_token, VFIO_VF_TOKEN_LEN);
	uuid_unparse(device_feature->data, uuid_string);
	printf(" [%s] ", uuid_string);

	ret = ioctl(vfio_dev_fd, VFIO_DEVICE_FEATURE, device_feature);
	free(device_feature);
	if (ret) {
		printf("ERR:VFIO: Fail to set VFIO_DEVICE_FEATURE with UUID token\n");
		return -1;
	}
	printf("Success\n");
	return 0;
#else
	printf("ERR:VFIO: VFIO_DEVICE_FEATURE IOCTL is not supported in OS\n");
	return -1;
#endif
}

void *vfio_get_bar0_mapping(const char *pci_addr, unsigned int bar_size,
										hw_device *hwd)
{
	int ret, groupid;
	int vfio_container_fd, vfio_group_fd, vfio_dev_fd;
	char path[PATH_MAX];
	void *map = NULL;

	struct vfio_group_status group_status = {
		.argsz = sizeof(group_status)
	};

	struct vfio_device_info device_info = {
		.argsz = sizeof(device_info)
	};

	struct vfio_region_info region_info = {
		.argsz = sizeof(region_info)
	};

	vfio_container_fd = open("/dev/vfio/vfio", O_RDWR);
	if (vfio_container_fd < 0) {
		printf("ERR:VFIO: Failed to open /dev/vfio/vfio, %d (%s)\n",
				vfio_container_fd, strerror(errno));
		goto error0;
	}

	groupid = vfio_get_device_groupid(pci_addr);
	if (groupid == -1) {
		printf("ERROR:VFIO: Failed to get groupid\n");
		goto error1;
	}
	printf("INFO:VFIO: Using PCI device [%s] in group %d\n", pci_addr, groupid);

	snprintf(path, sizeof(path), "/dev/vfio/%d", groupid);
	vfio_group_fd = open(path, O_RDWR);
	if (vfio_group_fd < 0) {
		printf("ERR:VFIO: Failed to open %s, %d (%s)\n",
				path, vfio_group_fd, strerror(errno));
		printf("ERR:VFIO: Device [%s] not bind to vfio-pci driver\n",
				pci_addr);
		goto error1;
	}

	ret = ioctl(vfio_group_fd, VFIO_GROUP_GET_STATUS, &group_status);
	if (ret) {
		printf("ERR:VFIO: ioctl(VFIO_GROUP_GET_STATUS) failed\n");
		goto error2;
	}

	if (!(group_status.flags & VFIO_GROUP_FLAGS_VIABLE)) {
		printf("ERROR:VFIO: Group not viable, are all devices attached to vfio?\n");
		goto error2;
	}

	/* NOTE: set container ioctl will attach the vfio_group_fd
	* to vfio_container_fd, will not override the vfio_conatiner_fd
	*/
	ret = ioctl(vfio_group_fd, VFIO_GROUP_SET_CONTAINER, &vfio_container_fd);
	if (ret) {
		printf("ERR:VFIO: Failed to set group container\n");
		goto error2;
	}

	ret = ioctl(vfio_container_fd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU);
	if (ret) {
		printf("ERR:VFIO: Failed to set IOMMU\n");
		goto error2;
	}

	vfio_dev_fd = ioctl(vfio_group_fd, VFIO_GROUP_GET_DEVICE_FD, pci_addr);
	if (vfio_dev_fd < 0) {
		printf("ERR:VFIO: Failed to get device %s\n", pci_addr);
		goto error2;
	}

	if (ioctl(vfio_dev_fd, VFIO_DEVICE_GET_INFO, &device_info)) {
		printf("ERR:VFIO: Failed to get device info\n");
		goto error2;
	}

	/* Configure VFIO token */
	if (hwd->vfio_mode) {
		if (vfio_set_token(hwd, vfio_dev_fd) < 0) {
			printf("ERR:VFIO: Fail to set VFIO_DEVICE_FEATURE with UUID token\n");
			goto error3;
		}
	}

	/* Map BAR0 region */
	region_info.index = 0;
	if (ioctl(vfio_dev_fd, VFIO_DEVICE_GET_REGION_INFO, &region_info)) {
		printf("ERR:VFIO: Failed to get region(%d) info\n", region_info.index);
		goto error3;
	}

	if (region_info.flags & VFIO_REGION_INFO_FLAG_MMAP) {
		map = mmap(NULL, (size_t)region_info.size,
				PROT_READ | PROT_WRITE, MAP_SHARED, vfio_dev_fd,
				(off_t)region_info.offset);
		if (map == MAP_FAILED) {
			printf("ERR:VFIO: mmap failed\n");
			map = NULL; /* MAP_FAILED is not equal to NULL */
			goto error3;
		}
	}

error3:
	close(vfio_dev_fd);
error2:
	close(vfio_group_fd);
error1:
	close(vfio_container_fd);
error0:
	return map;
}

static void *
sysfs_get_bar0_mapping(const char *pci_addr, unsigned int bar_size)
{
	char bar0path[PATH_MAX];
	int bar0addrfd;
	void *map;

	snprintf(bar0path, sizeof(bar0path),
			"%s/%s/%s", SYS_DIR, pci_addr, BAR0_FILE);
	bar0addrfd = open(bar0path, O_RDWR | O_SYNC);
	if (bar0addrfd < 0) {
		printf("\nFailed to open BAR %s\n", bar0path);
		exit(1);
	}
	map = mmap(0, bar_size, PROT_READ | PROT_WRITE, MAP_SHARED,
			bar0addrfd, 0);
	close(bar0addrfd);
	return map;
}

static void *
get_bar0_mapping(const char *pci_addr, unsigned int bar_size, hw_device *device)
{
	return (device->vfio_mode) ? vfio_get_bar0_mapping(pci_addr, bar_size, device) :
			sysfs_get_bar0_mapping(pci_addr, bar_size);
}

static unsigned long
get_file_val(const char *file_path)
{
	char content[BUFSIZ];
	FILE *f;

	f = fopen(file_path, "r");
	if (NULL == f) {
		printf("\nFailed to open %s\n", file_path);
		exit(1);
	}
	if (fgets(content, sizeof(content), f) == NULL) {
		fclose(f);
		return false;
	}
	fclose(f);
	const unsigned long content_val = strtoul(content, NULL, 16);

	return content_val;
}

static bool
get_device_id(hw_device *device, const char *location)
{
	unsigned long vendor_id = -1, device_id = -1;
	struct dirent *dirent;
	DIR *dir;
	char pci_path[PATH_MAX];

	snprintf(pci_path, sizeof(pci_path), "%s/%s", SYS_DIR, location);
	dir = opendir(pci_path);
	if (dir == NULL) {
		printf("Failed to open %s (%s)\n", pci_path, strerror(errno));
		return false;
	}

	while ((dirent = readdir(dir)) != NULL) {
		char file_path[PATH_MAX];

		/* Omit Current Directory & Previous Directory Lookup */
		if (strncmp(dirent->d_name, CUR_DIR,
				strlen(dirent->d_name)) == 0 ||
				strncmp(dirent->d_name, PREV_DIR,
				strlen(dirent->d_name)) == 0)
			continue;

		/* Set filepath as next PCI folder (xxxx:xx:xx.x) */
		int snprintf_ret =  snprintf(file_path, sizeof(file_path), "%s/%s",
				pci_path, dirent->d_name);
		if (snprintf_ret < 0)
			printf("Failed to format PCI path\n");

		/* Get Device ID */
		if (strncmp(dirent->d_name, DEVICE_FILE,
				strlen(dirent->d_name)) == 0 &&
				dirent->d_type == DT_REG)
			device_id = get_file_val(file_path);

		/* Get Vendor ID */
		if (strncmp(dirent->d_name, VENDOR_FILE,
				strlen(dirent->d_name)) == 0 &&
				dirent->d_type == DT_REG)
			vendor_id = get_file_val(file_path);
	}

	closedir(dir);
	/* Check if device is found */
	return (vendor_id == device->vendor_id &&
			device_id == device->device_id);
}

static int
probe_pci_bus(hw_device *device, char **found_devices)
{
	struct dirent *dirent;
	DIR *dir;
	int num_devices = 0;

	/* Locate PCI Devices */
	dir = opendir(SYS_DIR);
	if (dir == NULL)
		return -1;

	/* Iterate Through Directories */
	while ((dirent = readdir(dir)) != NULL) {

		/* Omit Current Directory and Previous Directory Lookup */
		if (strncmp(dirent->d_name, CUR_DIR,
				strlen(dirent->d_name)) == 0 ||
				strncmp(dirent->d_name, PREV_DIR,
						strlen(dirent->d_name)) == 0)
			continue;
		/* Check if current device matches requested device */
		if (get_device_id(device, dirent->d_name)) {
			found_devices[num_devices] =
					(char *) malloc(PCI_STR_SIZE);
			/* Copy PCI slot of device */
			strncpy(found_devices[num_devices], dirent->d_name,
					sizeof(device->pci_address) - NULL_PAD);
			num_devices++;
		}
	}

	return num_devices;
}

static int
match_device(char *pci_address, char **found_devices, int num_devices)
{
	int i;
	for (i = 0; i < num_devices; i++) {
		if (!strncmp(pci_address, found_devices[i],
				sizeof(pci_address) - NULL_PAD))
			return 0;
	}

	printf("Given PCI ID is not available\n");
	return -1;
}

static int
select_device(hw_device *device, char **found_devices, int num_devices)
{
	int i, selected;
	/* If more than one device found, get user input on which to use */
	if (num_devices >= 2) {
		printf("More than one device found. Please select which device you would like to use from the list:\n");

		/*Print PCI Slots */
		for (i = 0; i < num_devices; i++)
			printf("[%i]: %s\n", i+1, found_devices[i]);

		printf("> ");
		scanf("%d", &selected);
		if (selected >= 1 && selected <= num_devices) {
			strncpy(device->pci_address, found_devices[selected-1],
					sizeof(device->pci_address) - NULL_PAD);
			return 0;
		}

		printf("Invalid Option, please try again..\n");
		return -1;
	}

	strncpy(device->pci_address, found_devices[0],
			sizeof(device->pci_address) - NULL_PAD);
	return 0;
}

int set_device(hw_device *device)
{
	if (strcasecmp(device->device_name, "acc100") == 0) {
		device->vendor_id = ACC100_VENDOR_ID;
		device->device_id = ACC100_DEVICE_ID;
		device->conf = acc100_configure;

		if (device->config_file == NULL) {
			device->config_file =
					"acc100/acc100_config_2vf_4g5g.cfg";
		}
		return 0;
	}

	if (strcasecmp(device->device_name, "fpga_lte") == 0) {
		device->vendor_id = FPGA_LTE_FEC_VENDOR_ID;
		device->device_id = FPGA_LTE_FEC_DEVICE_ID;
		device->conf = fpga_lte_configure;
		if (device->config_file == NULL) {
			device->config_file = "fpga_lte/fpga_lte_config.cfg";
		}
		return 0;
	}

	if (strcasecmp(device->device_name, "fpga_5gnr") == 0) {
		device->vendor_id = FPGA_5GNR_FEC_VENDOR_ID;
		device->device_id = FPGA_5GNR_FEC_DEVICE_ID;
		device->conf = fpga_5gnr_configure;
		if (device->config_file == NULL) {
			device->config_file =
					"fpga_5gnr/fpga_5gnr_config.cfg";
		}
		return 0;
	}
	return -1;
}

static void
print_helper(const char *prgname)
{
	printf("Usage: %s DEVICE_NAME [-h] [-a] [-c CFG_FILE] [-p PCI_ID] [-v VFIO_TOKEN]\n\n"
			" DEVICE_NAME \t specifies device to configure [FPGA_LTE or FPGA_5GNR or ACC100]\n"
			" -c CFG_FILE \t specifies configuration file to use\n"
			" -p PCI_ID \t specifies PCI ID of device to configure\n"
			" -a \t\t configures all PCI devices matching the given DEVICE_NAME\n"
			" -v VFIO_TOKEN \t VFIO_TOKEN is UUID formatted VFIO VF token required when bound with vfio-pci\n"
			" -h \t\t prints this helper\n\n", prgname);
}

static int
bbdev_parse_args(int argc, char **argv,
		struct hw_device *device)
{
	int opt;
	char *prgname = argv[0];
	if (argc == 1) {
		print_helper(prgname);
		return 1;
	}
	device->device_name = argv[1];

	while ((opt = getopt(argc, argv, "c:p:v:ah")) != -1) {
		switch (opt) {
		case 'c':
			device->config_file = optarg;
			break;

		case 'p':
			strncpy(device->pci_address, optarg,
					sizeof(device->pci_address) - NULL_PAD);
			break;

		case 'a':
			device->config_all = 1;
			break;

		case 'v':
			device->vfio_mode = 1;
			if (uuid_parse(optarg, device->vfio_vf_token) < 0) {
				printf("UUID input: [%s]\n", optarg);
				return 1;
			}
			break;

		case 'h':
		default:
			print_helper(prgname);
			return 1;
		}
	}
	return 0;
}

static void daemonize(void)
{
	pid_t pid, sid;

	pid = fork();
	if (pid < 0)
		exit(EXIT_FAILURE);

	/* exit the parent */
	if (pid > 0)
		exit(EXIT_SUCCESS);

	umask(0);

	/* do not let the child to become orphan */
	sid = setsid();
	if (sid < 0)
		exit(EXIT_FAILURE);

	close(STDIN_FILENO);
	close(STDOUT_FILENO);
	close(STDERR_FILENO);

	/* a big infinite loop, kill the process to stop */
	while (1)
		sleep(SLEEP_30SEC);
}

static int
configure_device(hw_device *device)
{
	unsigned int bar_size = 0x1000;
	if (device->device_id == ACC100_DEVICE_ID)
		bar_size = 0x1000000;
	/* Get BAR0 Mapping for device */
	void *bar0addr = get_bar0_mapping(device->pci_address, bar_size, device);
	if (bar0addr == NULL)
		return -1;

	/* Call device specific configuration function */
	if (device->conf(bar0addr, device->config_file) == 0) {
		printf("%s PF [%s] configuration complete!\n\n",
				device->device_name, device->pci_address);
		return 0;
	}

	printf("Configuration error!!\n");
	return -1;
}

int
main(int argc, char *argv[])
{
	int i, ret, num_devices;
	hw_device device;
	char *found_devices[10];
	printf("== pf_bb_config Version #VERSION_STRING# ==\n");

	memset(&device, 0, sizeof(device));

	if (bbdev_parse_args(argc, argv, &device) > 0)
		return 0;

	/* Set Device Info */
	ret = set_device(&device);
	if (ret != 0) {
		printf("Device '%s' is not supported.\n", device.device_name);
		return 0;
	}

	/* Check if device is installed */
	num_devices = probe_pci_bus(&device, found_devices);
	if (num_devices == 0) {
		printf("No devices found!!\n");
		return -1;
	} else if (num_devices < 0) {
		return num_devices;
	}

	if (device.pci_address[0] != 0) {
		if (match_device(device.pci_address, found_devices,
				num_devices) < 0)
			return -1;
	}

	if (device.config_all) {
		for (i = 0; i < num_devices; i++) {
			strncpy(device.pci_address, found_devices[i],
					sizeof(device.pci_address) - NULL_PAD);
			ret = configure_device(&device);
		}
	} else {
		select_device(&device, found_devices, num_devices);
		ret = configure_device(&device);
	}

	/* Free memory for stored PCI slots */
	for (i = 0; i < num_devices; i++)
		free(found_devices[i]);

	if (ret == 0 && device.vfio_mode) {
		printf("Running in daemon mode for VFIO VF token\n");
		daemonize();
	}

	return ret;
}
