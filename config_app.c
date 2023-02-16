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
#include <sys/eventfd.h>

#include "bb_acc.h"
#include "bb_acc_log.h"

static void *
uio_get_bar0_mapping(const char *pci_addr,
		unsigned int bar_size, void *dev)
{
	char bar0path[PATH_MAX];
	int bar0addrfd;
	void *map;

	snprintf(bar0path, sizeof(bar0path),
			"%s/%s/%s", SYS_DIR, pci_addr, BAR0_FILE);
	bar0addrfd = open(bar0path, O_RDWR | O_SYNC);
	if (bar0addrfd < 0) {
		LOG(ERR, "Failed to open BAR %s", bar0path);
		exit(1);
	}
	map = mmap(0, bar_size, PROT_READ | PROT_WRITE, MAP_SHARED,
			bar0addrfd, 0);
	close(bar0addrfd);
	if (map == MAP_FAILED) {
		printf("ERR:SYSFS: mmap failed\n");
		map = NULL; /* MAP_FAILED is not equal to NULL */
	}
	return map;
}

static unsigned long
get_file_val(const char *file_path)
{
	char content[BUFSIZ];
	FILE *f;

	f = fopen(file_path, "r");
	if (NULL == f) {
		LOG(ERR, "Failed to open %s", file_path);
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
		LOG(ERR, "Failed to open %s (%s)", pci_path, strerror(errno));
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
			LOG(ERR, "Failed to format PCI path");

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

	closedir(dir);
	return num_devices;
}

static int
match_device(char *pci_address, char **found_devices, int num_devices)
{
	int i, null_prefix = 0;

	/* Handle the case when the node prefix 0000: is not included */
	if (strlen(pci_address) == 7)
		null_prefix = 5;

	for (i = 0; i < num_devices; i++) {
		if (!strncmp(pci_address, found_devices[i] + null_prefix,
				sizeof(pci_address) - NULL_PAD))
			return 0;
	}

	printf("ERR: Given PCI ID is not available %s\n", pci_address);
	return -1;
}

static int
select_device(hw_device *device, char **found_devices, int num_devices)
{
	int i, selected, ret;
	/* If more than one device found, get user input on which to use */
	if (num_devices >= 2) {
		printf("More than one device found. Please select which device you would like to use from the list:\n");

		/*Print PCI Slots */
		for (i = 0; i < num_devices; i++)
			printf("[%i]: %s\n", i+1, found_devices[i]);

		printf("> ");
		ret = scanf("%d", &selected);
		if (selected >= 1 && selected <= num_devices && ret == 1) {
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
		device->ops.conf  = acc100_configure;
		device->ops.device_data  = acc100_device_data;
		device->bar_size = 0x1000000;

		if (device->vfio_mode) {
			device->ops.open = vfio_device_open;
			device->ops.get_bar0_addr = vfio_get_bar0_mapping;
		} else {
			device->ops.get_bar0_addr = uio_get_bar0_mapping;
		}

		if (device->config_file == NULL) {
			device->config_file =
					"acc100/acc100_config_2vf_4g5g.cfg";
		}
		return 0;
	}

	if (strcasecmp(device->device_name, "acc200") == 0) {
		device->vendor_id = ACC200_VENDOR_ID;
		device->device_id = ACC200_DEVICE_ID;
		device->ops.conf = acc200_configure;
		device->ops.device_data  = acc200_device_data;
		device->bar_size = 0x1000000;
		if (device->vfio_mode) {
			device->ops.open = vfio_device_open;
			device->ops.flr = vfio_device_reset;
			device->ops.cluster_reset = acc200_cluster_reset;
			device->ops.enable_intr = vfio_enable_intr;
			device->ops.disable_intr = vfio_disable_intr;
			device->ops.dev_enable_intr = acc200_enable_intr;
			device->ops.dev_disable_intr = acc200_disable_intr;
			device->ops.get_bar0_addr = vfio_get_bar0_mapping;
			device->ops.dev_isr = acc200_irq_handler;
			device->vfio_int_mode = VFIO_PCI_MSI_IRQ_INDEX;
			device->auto_reconfig_on_fatal_error = DEVICE_RESET_AUTO_RECONFIG;
			device->device_reset_using_flr = DEVICE_RESET_USING_CLUSTER;
			device->info_ring_total_size = BB_ACC_INFO_RING_SIZE;
		} else {
			device->ops.get_bar0_addr = uio_get_bar0_mapping;
		}
		if (device->config_file == NULL) {
			device->config_file = "acc200/acc200_config.cfg";
		}
		return 0;
	}

	if (strcasecmp(device->device_name, "fpga_lte") == 0) {
		device->vendor_id = FPGA_LTE_FEC_VENDOR_ID;
		device->device_id = FPGA_LTE_FEC_DEVICE_ID;
		device->ops.conf = fpga_lte_configure;
		device->bar_size = 0x1000;
		device->ops.get_bar0_addr = uio_get_bar0_mapping;

		if (device->config_file == NULL) {
			device->config_file = "fpga_lte/fpga_lte_config.cfg";
		}
		return 0;
	}

	if (strcasecmp(device->device_name, "fpga_5gnr") == 0) {
		device->vendor_id = FPGA_5GNR_FEC_VENDOR_ID;
		device->device_id = FPGA_5GNR_FEC_DEVICE_ID;
		device->ops.conf = fpga_5gnr_configure;
		device->bar_size = 0x1000;
		device->ops.get_bar0_addr = uio_get_bar0_mapping;

		if (device->config_file == NULL) {
			device->config_file =
					"fpga_5gnr/fpga_5gnr_config.cfg";
		}
		return 0;
	}

	if (strcasecmp(device->device_name, "agx100") == 0) {
		device->vendor_id = AGX100_VENDOR_ID;
		device->device_id = AGX100_DEVICE_ID;
		device->ops.conf = agx100_configure;
		device->bar_size = 0x1000;

		if (device->vfio_mode) {
			device->ops.open = vfio_device_open;
			device->ops.get_bar0_addr = vfio_get_bar0_mapping;
		} else {
			device->ops.get_bar0_addr = uio_get_bar0_mapping;
		}

		if (device->config_file == NULL) {
			device->config_file =
					"agx100/agx100_config_1vf.cfg";
		}
		return 0;
	}
	return -1;
}

static void
print_helper(const char *prgname)
{
	printf("Usage: %s DEVICE_NAME [-h] [-c CFG_FILE] [-p PCI_ID] [-v VFIO_TOKEN]\n\n"
			" DEVICE_NAME \t specifies device to configure [FPGA_LTE or FPGA_5GNR or ACC100]\n"
			" -c CFG_FILE \t specifies configuration file to use\n"
			" -p PCI_ID \t specifies PCI ID of device to configure ([0000:]51:00.0)\n"
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

	while ((opt = getopt(argc, argv, "c:p:v:h")) != -1) {
		switch (opt) {
		case 'c':
			device->config_file = optarg;
			break;

		case 'p':
			strncpy(device->pci_address, optarg,
					sizeof(device->pci_address) - NULL_PAD);
			device->pci_address[PCI_STR_SIZE - 1] = 0;
			break;

		case 'v':
			device->vfio_mode = 1;
			if (vfio_uuid_parse(optarg, device->vfio_vf_token) < 0)
				return 1;
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

	if ((device->ops.open) && (device->ops.open(device))) {
		LOG(ERR, "device(%s) open failed", device->device_name);
		return -1;
	}

	/* Get BAR0 Mapping for device */
	if (device->ops.get_bar0_addr) {
		device->bar0Addr = device->ops.get_bar0_addr(device->pci_address,
				device->bar_size, device);
		if (device->bar0Addr == NULL)
			return -1;
	} else
		return -1;

	/* Call device specific configuration function */
	if (device->ops.conf(device, device->bar0Addr, device->config_file) == 0) {
		LOG(INFO, "%s PF [%s] configuration complete!",
				device->device_name, device->pci_address);
	} else {
		LOG(ERR, "%s PF [%s] configuration failed!",
				device->device_name, device->pci_address);
		return -1;
	}

	/* enable intr if supported, applicable only in case of vfio */
	if ((device->ops.enable_intr) && (device->ops.enable_intr(device))) {
		LOG(ERR, "Enable interrupts failed");
		return -1;
	}

	bb_acc_set_all_device_status(device, RTE_BBDEV_DEV_CONFIGURED);

	LOG(DEBUG, "Done");
	return 0;
}

int
main(int argc, char *argv[])
{
	int i, ret = 0, num_devices;
	hw_device device;
	char *found_devices[10];
	char logFile[BB_ACC_LOG_FILE_LEN];

	memset(&device, 0, sizeof(device));

	if (bbdev_parse_args(argc, argv, &device) > 0)
		return 0;

	event_init_fd();
	printf("== pf_bb_config Version #VERSION_STRING# ==\n");

	/* Set Device Info */
	ret = set_device(&device);
	if (ret != 0) {
		printf("ERR: Device '%s' is not supported\n", device.device_name);
		return 0;
	}

	/* Check if device is installed */
	num_devices = probe_pci_bus(&device, found_devices);
	if (num_devices == 0) {
		printf("ERR: No devices found!!\n");
		return -1;
	} else if (num_devices < 0) {
		return num_devices;
	}

	if (device.pci_address[0] != 0) {
		if (match_device(device.pci_address, found_devices,
				num_devices) < 0)
			return -1;
	}

	select_device(&device, found_devices, num_devices);
	sprintf(logFile, "%s/pf_bb_cfg_%s.log", BB_ACC_DEFAULT_LOG_PATH,
			device.pci_address);
	if (bb_acc_logInit(logFile, BB_ACC_MAX_LOG_FILE_SIZE, INFO, device.vfio_mode)) {
		printf("ERR: Logfile init failed\n");
		return -1;
	}
	ret = configure_device(&device);

	/* Free memory for stored PCI slots */
	for (i = 0; i < num_devices; i++)
		free(found_devices[i]);

	if (ret == 0 && device.vfio_mode) {
		LOG(INFO, "Running in daemon mode for VFIO VF token");
		printf("Log file = /var/log/pf_bb_cfg_%s.log\n", device.pci_address);
		daemonize(&device);
	}

	return ret;
}
