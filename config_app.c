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
} hw_device;

static void *
get_bar0_mapping(const char *pci_addr, unsigned int bar_size)
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
		snprintf(file_path, sizeof(file_path), "%s/%s",
				pci_path, dirent->d_name);

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
			device->config_file = getenv("ACC100_CONFIG_FILE");
			if (device->config_file == NULL)
				device->config_file =
						"acc100/acc100_config.cfg";
		}
		return 0;
	}

	if (strcasecmp(device->device_name, "fpga_lte") == 0) {
		device->vendor_id = FPGA_LTE_FEC_VENDOR_ID;
		device->device_id = FPGA_LTE_FEC_DEVICE_ID;
		device->conf = fpga_lte_configure;
		if (device->config_file == NULL) {
			device->config_file = getenv("FPGA_LTE_CONFIG_FILE");
			if (device->config_file == NULL)
				device->config_file =
						"fpga_lte/fpga_lte_config.cfg";
		}
		return 0;
	}

	if (strcasecmp(device->device_name, "fpga_5gnr") == 0) {
		device->vendor_id = FPGA_5GNR_FEC_VENDOR_ID;
		device->device_id = FPGA_5GNR_FEC_DEVICE_ID;
		device->conf = fpga_5gnr_configure;
		if (device->config_file == NULL) {
			device->config_file = getenv("FPGA_5GNR_CONFIG_FILE");
			if (device->config_file == NULL)
				device->config_file =
						"fpga_5gnr/fpga_5gnr_config.cfg"
						;
		}
		return 0;
	}
	return -1;
}

static void
print_helper(const char *prgname)
{
	printf("Usage: %s DEVICE_NAME [-h] [-a] [-c CFG_FILE] [-p PCI_ID]\n\n"
			" DEVICE_NAME \t specifies device to configure [FPGA_LTE or FPGA_5GNR or ACC100]\n"
			" -c CFG_FILE \t specifies configuration file to use\n"
			" -p PCI_ID \t specifies PCI ID of device to configure\n"
			" -a \t\t configures all PCI devices matching the given DEVICE_NAME\n"
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

	while ((opt = getopt(argc, argv, "c:p:ah")) != -1) {
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

		case 'h':
		default:
			print_helper(prgname);
			return 1;
		}
	}
	return 0;
}

static int
configure_device(hw_device *device)
{
	unsigned int bar_size = 0x1000;
	if (device->device_id == ACC100_DEVICE_ID)
		bar_size = 0x1000000;
	/* Get BAR0 Mapping for device */
	void *bar0addr = get_bar0_mapping(device->pci_address, bar_size);

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

	ret = 0;
	if (device.config_all) {
		for (i = 0; i < num_devices; i++) {
			strncpy(device.pci_address, found_devices[i],
					sizeof(device.pci_address) - NULL_PAD);
			ret = configure_device(&device);
			if (ret != 0) {
				break;
			}
		}
	} else {
		select_device(&device, found_devices, num_devices);
		ret = configure_device(&device);
	}

	/* Free memory for stored PCI slots */
	for (i = 0; i < num_devices; i++)
		free(found_devices[i]);

	return ret;
}
