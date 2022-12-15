/******************************************************************************
*
*   Copyright (c) 2022 Intel.
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


void vfio_device_close(hw_device *dev)
{
	hw_device *hwd = (hw_device *)dev;

	if (hwd != NULL)
		close(hwd->vfio_dev_fd);
}

int vfio_read(int fd, void *buf, size_t len, off_t off)
{
	if (pread(fd, buf, len, off) != len) {
		LOG(ERR, "pread(off=%#lx len=%lu)", off, len);
		return -1;
	}

	return 0;
}

int vfio_write(int fd, void *buf, size_t len, off_t off)
{
	if (pwrite(fd, buf, len, off) != len) {
		LOG(ERR, "pwrite(off=%#lx len=%lu)", off, len);
		return -1;
	}

	return 0;
}

int vfio_device_reset(void *dev)
{
	hw_device *hwd = (hw_device *)dev;

	LOG(DEBUG, "VFIO device reset:");

	if (ioctl(hwd->vfio_dev_fd, VFIO_DEVICE_RESET)) {
		LOG(DEBUG, "VFIO_DEVICE_RESET ioctl failed");
		return -1;
	}

	LOG(DEBUG, "Done");

	return 0;
}

static int vfio_set_bus_master(hw_device *hwd)
{
	int i;
	uint8_t config[VFIO_PCI_CONFIG_REGION_SIZE];

	for (i = 0; i < hwd->num_regions; i++) {
		struct vfio_region_info reg = {
				.argsz = sizeof(reg), .index = i
		};

		if (ioctl(hwd->vfio_dev_fd, VFIO_DEVICE_GET_REGION_INFO, &reg))
			continue;

		if (i == VFIO_PCI_CONFIG_REGION_INDEX) {
			uint16_t *vendor;
			uint16_t *cmd;

			if (vfio_read(hwd->vfio_dev_fd, config, sizeof(config), reg.offset))
				return -1;

			vendor = (uint16_t *)(config + PCI_VENDOR_ID);
			cmd = (uint16_t *)(config + PCI_COMMAND);

			if (*vendor == 0xffff) {
				LOG(ERR, "device in bad state");
				return -1;
			}

			*cmd |= PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
			if (vfio_write(hwd->vfio_dev_fd, cmd,
					sizeof(*cmd), reg.offset + PCI_COMMAND))
				return -1;

			if (vfio_read(hwd->vfio_dev_fd, cmd,
					sizeof(*cmd), reg.offset + PCI_COMMAND))
				return -1;

			LOG(DEBUG, "vendor=%#x cmd=%#x device=%#x rev=%d",
					*vendor, *cmd,
					(uint32_t)*((uint16_t *)(config + PCI_DEVICE_ID)),
					(uint32_t)config[PCI_REVISION_ID]);

			break;
		}
	} /* loop on all regions */

	return 0;
}

static int vfio_enable_msi(hw_device *hwd)
{
	int len;
	int32_t *pfd;
	char  set_irq_buf[VFIO_IRQ_SET_BUF_LEN];
	struct vfio_irq_set *irqs;

	LOG(DEBUG, "MSI IRQ event registration");

	/* check if already enabled then do nothing */
	if (hwd->irq_event_fd > 0)
		return 0;

	hwd->irq_event_fd = eventfd(0, EFD_CLOEXEC);
	if (hwd->irq_event_fd < 0) {
		LOG(ERR, "Failed to get eventfd for vfio interrupt");
		return -1;
	}
	LOG(DEBUG, "irq_event_fd: %d", hwd->irq_event_fd);

	len = sizeof(set_irq_buf);

	irqs = (struct vfio_irq_set *)set_irq_buf;
	irqs->argsz = len;
	irqs->index = VFIO_PCI_MSI_IRQ_INDEX;
	irqs->flags = VFIO_IRQ_SET_DATA_EVENTFD | VFIO_IRQ_SET_ACTION_TRIGGER;
	irqs->start = 0;
	irqs->count = 1;
	pfd = (int *)&irqs->data;
	*pfd = hwd->irq_event_fd;

	if (ioctl(hwd->vfio_dev_fd, VFIO_DEVICE_SET_IRQS, irqs)) {
		LOG(ERR, "ioctl VFIO_DEVICE_SET_IRQS error %d", errno);
		close(hwd->irq_event_fd);
		hwd->irq_event_fd = 0;
		return -1;
	}

	LOG(DEBUG, "irq eventfd registration success");
	return 0;
}

static int vfio_disable_msi(hw_device *hwd)
{
	int len;
	char  set_irq_buf[VFIO_IRQ_SET_BUF_LEN];
	struct vfio_irq_set *irqs;

	LOG(DEBUG, "MSI IRQ event deregistration");

	/* if not registred, do nothing */
	if (hwd->irq_event_fd == 0)
		return 0;

	LOG(DEBUG, "irq_event_fd: %d", hwd->irq_event_fd);

	len = sizeof(struct vfio_irq_set);

	irqs = (struct vfio_irq_set *)set_irq_buf;
	irqs->argsz = len;
	irqs->index = VFIO_PCI_MSI_IRQ_INDEX;
	irqs->flags = VFIO_IRQ_SET_DATA_NONE | VFIO_IRQ_SET_ACTION_TRIGGER;
	irqs->start = 0;
	irqs->count = 1;

	if (ioctl(hwd->vfio_dev_fd, VFIO_DEVICE_SET_IRQS, irqs)) {
		LOG(ERR, "ioctl VFIO_DEVICE_SET_IRQS error");
		return -1;
	}

	LOG(DEBUG, "irq eventfd deregistration success");
	return 0;
}

static void vfio_intr_handler(int event_fd, void *dev)
{
	hw_device *hwd = (hw_device *)dev;
	int len;
	uint64_t buf;

	LOG(DEBUG, "VFIO interrupt handler:");

	do {
		len = read(event_fd, &buf, sizeof(buf));
		LOG(DEBUG, "event val: %lu, len:%d", buf, len);

		if (len < 0) {
			LOG(ERR, "vfio event irq recv() failed");
			return;
		}
	} while (len != sizeof(buf));

	if (hwd->ops.dev_isr)
		hwd->ops.dev_isr(hwd);
}

int vfio_enable_intr(void *dev)
{
	hw_device *hwd = (hw_device *)dev;

	LOG(DEBUG, "VFIO enable interrupts");

	if (!hwd->vfio_mode)
		return 0;

	switch (hwd->vfio_int_mode) {
	case VFIO_PCI_MSI_IRQ_INDEX:
		if (vfio_set_bus_master(hwd)) {
			LOG(ERR, "vfio enable bus master failed");
			return -1;
		}

		if (vfio_enable_msi(hwd)) {
			LOG(ERR, "vfio enable MSI intr failed");
			return -1;
		}

		if ((hwd->ops.dev_enable_intr) && (hwd->ops.dev_enable_intr(hwd))) {
			LOG(ERR, "vfio enable %s device intr failed", hwd->device_name);
			return -1;
		}

		/* register isr callback for intr event */
		event_add_fd(hwd->irq_event_fd, vfio_intr_handler, hwd);
		break;

	case VFIO_PCI_INTX_IRQ_INDEX:
	case VFIO_PCI_MSIX_IRQ_INDEX:
	case VFIO_PCI_ERR_IRQ_INDEX:
	default:
		LOG(INFO, "Not supported irq index (%d)", hwd->vfio_int_mode);
		break;
	}

	LOG(DEBUG, "Done");
	return 0;
}

int vfio_disable_intr(void *dev)
{
	hw_device *hwd = (hw_device *)dev;

	LOG(DEBUG, "VFIO disable interrupts");

	if (!hwd->vfio_mode)
		return 0;

	switch (hwd->vfio_int_mode) {
	case VFIO_PCI_MSI_IRQ_INDEX:
		if ((hwd->ops.dev_disable_intr) && (hwd->ops.dev_disable_intr(hwd))) {
			LOG(ERR, "vfio enable %s device intr failed", hwd->device_name);
			return -1;
		}

		if (vfio_disable_msi(hwd))
			LOG(ERR, "vfio disable MSI intr failed");

		/* deregister isr callback for intr event */
		event_del_fd(hwd->irq_event_fd);
		close(hwd->irq_event_fd);
		hwd->irq_event_fd = 0;
		break;

	case VFIO_PCI_INTX_IRQ_INDEX:
	case VFIO_PCI_MSIX_IRQ_INDEX:
	case VFIO_PCI_ERR_IRQ_INDEX:
	default:
		LOG(INFO, "Not supported irq index (%d)", hwd->vfio_int_mode);
		break;
	}

	LOG(DEBUG, "Done");
	return 0;
}

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
		LOG(ERR, "VFIO: iommu_group error for %s", pci_addr);
		return -1;
	}
	group_path[len] = 0;

	group_name = basename(group_path);
	groupid = strtol(group_name, NULL, 10);
	if (groupid == 0) {
		LOG(ERR, "VFIO: Faild to read %s", group_path);
		return -1;
	}

	return groupid;
}

int vfio_uuid_parse(char *uuid_str, unsigned char *uuid)
{
	int i, inx;
	int high_n = 1;
	unsigned char tmp;

	if (strlen(uuid_str) != VFIO_VF_TOKEN_STR_LEN) {
		LOG(ERR, "uuid string len is wrong: %d", (int)strlen(uuid_str));
		return -1;
	}
	for (i = 0, inx = 0;
			((i < VFIO_VF_TOKEN_STR_LEN) && (inx < VFIO_VF_TOKEN_LEN));
			i++) {
		if (uuid_str[i] == '-') {
			if ((i ==  8) || (i == 13) || (i == 18) || (i == 23))
				continue;
		}
		if (!isxdigit(uuid_str[i])) {
			LOG(ERR, "Unknown char in uuid string");
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
		LOG(ERR, "memory allocaton failed");
		return -1;
	}

	/* Set the secret token shared between PF and VF */
	LOG(DEBUG, "Setting VFIO_DEVICE_FEATURE with UUID token");

	device_feature->argsz = sizeof(device_feature) + VFIO_VF_TOKEN_LEN;
	device_feature->flags = VFIO_DEVICE_FEATURE_SET |
			VFIO_DEVICE_FEATURE_PCI_VF_TOKEN;
	memcpy(device_feature->data, hwd->vfio_vf_token, VFIO_VF_TOKEN_LEN);
	uuid_unparse(device_feature->data, uuid_string);
	LOG(DEBUG, "[%s]", uuid_string);

	ret = ioctl(vfio_dev_fd, VFIO_DEVICE_FEATURE, device_feature);
	free(device_feature);
	if (ret) {
		LOG(ERR, "Fail to set VFIO_DEVICE_FEATURE with UUID token");
		return -1;
	}
	LOG(DEBUG, "Done");
	return 0;
#else
	LOG(ERR, "VFIO_DEVICE_FEATURE IOCTL is not supported in OS");
	return -1;
#endif
}

int vfio_device_open(void *dev)
{
	int   ret, groupid;
	int   vfio_container_fd, vfio_group_fd, vfio_dev_fd;
	char  path[PATH_MAX];

	hw_device *hwd = (hw_device *)dev;
	char *pci_addr = hwd->pci_address;

	struct vfio_group_status group_status = {
		.argsz = sizeof(group_status)
	};

	struct vfio_device_info device_info = {
		.argsz = sizeof(device_info)
	};

	struct vfio_iommu_type1_dma_map dma_map = {
		.argsz = sizeof(dma_map)
	};

	LOG(DEBUG, "PCI device: %s", pci_addr);

	if (!hwd->vfio_mode) {
		LOG(WARN, "not opened in vfio mode");
		return -1;
	}

	if (hwd->vfio_dev_fd) {
		LOG(DEBUG, "VFIO device already opened");
		return 0;
	}

	vfio_container_fd = open("/dev/vfio/vfio", O_RDWR);
	if (vfio_container_fd < 0) {
		LOG(ERR, "Failed to open /dev/vfio/vfio, %d (%s)",
				vfio_container_fd, strerror(errno));
		goto error0;
	}

	groupid = vfio_get_device_groupid(pci_addr);
	if (groupid == -1) {
		LOG(ERR, "Failed to get groupid");
		goto error1;
	}
	LOG(DEBUG, "VFIO: Using PCI device [%s] in group %d",
			pci_addr, groupid);

	snprintf(path, sizeof(path), "/dev/vfio/%d", groupid);
	vfio_group_fd = open(path, O_RDWR);
	if (vfio_group_fd < 0) {
		LOG(ERR, "Failed to open %s, %d (%s)",
				path, vfio_group_fd, strerror(errno));
		LOG(ERR, "Device [%s] not bind to vfio-pci driver",
				pci_addr);
		goto error1;
	}

	ret = ioctl(vfio_group_fd, VFIO_GROUP_GET_STATUS, &group_status);
	if (ret) {
		LOG(ERR, "ioctl(VFIO_GROUP_GET_STATUS) failed");
		goto error2;
	}

	if (!(group_status.flags & VFIO_GROUP_FLAGS_VIABLE)) {
		LOG(ERR, "Group not viable, are all devices attached to vfio?");
		goto error2;
	}

	/* NOTE: set container ioctl will attach the vfio_group_fd
	* to vfio_container_fd, will not override the vfio_conatiner_fd
	*/
	ret = ioctl(vfio_group_fd, VFIO_GROUP_SET_CONTAINER,
			&vfio_container_fd);
	if (ret) {
		LOG(ERR, "Failed to set group container");
		goto error2;
	}

	ret = ioctl(vfio_container_fd,
			VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU);
	if (ret) {
		LOG(ERR, "Failed to set IOMMU");
		goto error2;
	}

	if (hwd->info_ring_total_size) {
		/* Allocate some space and setup a DMA mapping */
		dma_map.vaddr = (uint64_t)mmap(0, hwd->info_ring_total_size,
				PROT_READ | PROT_WRITE,
				MAP_SHARED | MAP_ANONYMOUS, 0, 0);
		if (dma_map.vaddr == (uint64_t)MAP_FAILED) {
			LOG(ERR, "DMA mapping failed");
			goto error2;
		}
		dma_map.size = hwd->info_ring_total_size;
		dma_map.iova = dma_map.vaddr;
		dma_map.flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE;

		ret = ioctl(vfio_container_fd, VFIO_IOMMU_MAP_DMA, &dma_map);
		if (ret) {
			LOG(ERR, "Failed to set IOMMU_MAP_DMA");
			goto error2;
		}
		hwd->iommu_map_dma_vaddr = (void *)(dma_map.vaddr);
		hwd->iommu_map_dma_size  = hwd->info_ring_total_size;
		LOG(DEBUG, "VFIO selected IOVA mode 'VA'");
	}

	vfio_dev_fd = ioctl(vfio_group_fd,
			VFIO_GROUP_GET_DEVICE_FD, pci_addr);
	if (vfio_dev_fd < 0) {
		LOG(ERR, "Failed to get device %s", pci_addr);
		goto error2;
	}

	close(vfio_container_fd);
	close(vfio_group_fd);
	hwd->vfio_dev_fd = vfio_dev_fd;

	LOG(DEBUG, "Done, vfio_dev_fd: %d", vfio_dev_fd);
	return 0;

error2:
	close(vfio_group_fd);
error1:
	close(vfio_container_fd);
error0:
	return -1;
}

void *vfio_get_bar0_mapping(const char *pci_addr,
			unsigned int bar_size, void *dev)
{
	int vfio_dev_fd;
	void *map = NULL;
	hw_device *hwd = (hw_device *)dev;

	struct vfio_device_info device_info = {
		.argsz = sizeof(device_info)
	};

	struct vfio_region_info region_info = {
		.argsz = sizeof(region_info)
	};

	vfio_dev_fd = hwd->vfio_dev_fd;

	if (ioctl(vfio_dev_fd, VFIO_DEVICE_GET_INFO, &device_info)) {
		LOG(ERR, "Failed to get device info, dev fd: %d",
				vfio_dev_fd);
		return NULL;
	}
	LOG(DEBUG, "Device supports %d regions, %d irqs",
			device_info.num_regions, device_info.num_irqs);
	hwd->num_regions = device_info.num_regions;

	/* Configure VFIO token */
	if (vfio_set_token(hwd, vfio_dev_fd) < 0) {
		LOG(ERR, "Fail to set VFIO_DEVICE_FEATURE with UUID token");
		return NULL;
	}

	/* Map BAR0 region */
	region_info.index = 0;
	if (ioctl(vfio_dev_fd, VFIO_DEVICE_GET_REGION_INFO,
			&region_info)) {
		LOG(ERR, "Failed to get region(%d) info",
				region_info.index);
		return NULL;
	}

	if (region_info.flags & VFIO_REGION_INFO_FLAG_MMAP) {
		map = mmap(NULL, (size_t)region_info.size,
				PROT_READ | PROT_WRITE, MAP_SHARED, vfio_dev_fd,
				(off_t)region_info.offset);
		if (map == MAP_FAILED) {
			LOG(ERR, "mmap failed for BAR0 space");
			return NULL;
		}
	}

	/* Enable BME */
	vfio_set_bus_master(hwd);

	return map;
}

void *vfio_dma_alloc(void *dev, size_t size, uint64_t *phy)
{
	hw_device *hwd = (hw_device *)dev;

	if (hwd->iommu_map_dma_size < size)
		return NULL;

	/* using va as iova */
	*phy = (uint64_t)(hwd->iommu_map_dma_vaddr);

	return hwd->iommu_map_dma_vaddr;
}

void vfio_dma_free(void *virt, size_t size)
{
	if (!virt)
		return;
	/* In case of IOMMU, mapping is done at container level
	* free/unmap need to take care at device close.
	*/
}

