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

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include <unistd.h>
#include <errno.h>

#include "cfg_reader.h"
#include "bb_acc_reg_dump.h"
#include "bb_acc.h"
#include "bb_acc_log.h"

#define BBDEV_DEV_STATUS_COUNT 9

static uint32_t
reg_read(uint8_t *mmio_base, uint32_t offset)
{

	void *reg_addr = mmio_base + offset;
	uint32_t ret = *((volatile uint32_t *)(reg_addr));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	ret = __bswap_32(ret);
#endif
	return ret;
}

static void
reg_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
}

const char *
bb_acc_device_status_str(unsigned int status)
{
	static const char * const dev_sta_string[] = {
		"RTE_BBDEV_DEV_NOSTATUS",
		"RTE_BBDEV_DEV_NOT_SUPPORTED",
		"RTE_BBDEV_DEV_RESET",
		"RTE_BBDEV_DEV_CONFIGURED",
		"RTE_BBDEV_DEV_ACTIVE",
		"RTE_BBDEV_DEV_FATAL_ERR",
		"RTE_BBDEV_DEV_RESTART_REQ",
		"RTE_BBDEV_DEV_RECONFIG_REQ",
		"RTE_BBDEV_DEV_CORRECT_ERR",
	};

	if (status < BBDEV_DEV_STATUS_COUNT)
		return dev_sta_string[status];

	LOG(ERR, "Invalid device status");

	return NULL;
}

static int
bb_acc_reconfigure_device(hw_device *accel_dev)
{

	/* Call device specific configuration function */
	if (accel_dev->ops.conf(accel_dev, accel_dev->bar0Addr, accel_dev->config_file)) {
		LOG(ERR, "Reconfiguration failed");
		return -1;
	}

	/* enable intr if supported, applicable only in case of vfio */
	if ((accel_dev->ops.enable_intr) &&
			(accel_dev->ops.enable_intr(accel_dev))) {
		LOG(ERR, "Enable interrupts failed");
		return -1;
	}

	LOG(INFO, "%s PF [%s] Reconfiguration complete!",
			accel_dev->device_name, accel_dev->pci_address);

	bb_acc_set_all_device_status(accel_dev, RTE_BBDEV_DEV_CONFIGURED);

	return 0;
}

int
bb_acc_dev_reset_and_reconfig(void *dev)
{
	hw_device *accel_dev = (hw_device *)dev;

	if (dev == NULL) {
		LOG(ERR, "Device not set");
		return -1;
	}

	/* disable interrupts */
	if ((accel_dev->ops.disable_intr) &&
			(accel_dev->ops.disable_intr(accel_dev))) {
		LOG(ERR, "Disable interrupts failed");
		return -1;
	}

	/* reset */
	if ((accel_dev->ops.flr) &&
			(accel_dev->ops.flr(accel_dev))) {
		LOG(ERR, "Device reset failed");
	}

	/* re-init and configure the device */
	if (bb_acc_reconfigure_device(accel_dev))
		return -1;

	return 0;
}

int
bb_acc_cluster_reset_and_reconfig(void *dev)
{
	hw_device *accel_dev = (hw_device *)dev;

	LOG(DEBUG, "Cluster reset and reconfig");

	if (accel_dev == NULL)
		return 0;

	/* disable interrupts */
	if ((accel_dev->ops.disable_intr) &&
			(accel_dev->ops.disable_intr(accel_dev))) {
		LOG(ERR, "Disable interrupts failed");
		return -1;
	}

	if (accel_dev->ops.cluster_reset)
		accel_dev->ops.cluster_reset(accel_dev);
	else
		LOG(ERR, "Cluster reset is not supported");

	/* re-init and configure the device */
	if (bb_acc_reconfigure_device(accel_dev))
		return -1;

	LOG(DEBUG, "Done");
	return 0;
}

void
bb_acc_set_all_device_status(void *dev, unsigned int status)
{
	unsigned int i;
	hw_device *accel_dev = (hw_device *)dev;

	for (i = 0; i < BB_ACC_MAX_VFS; i++) {
		if (status == RTE_BBDEV_DEV_CONFIGURED) {
			/* After Hard reset the application needs to restart/reconfigure */
			if (accel_dev->dev_status[i] == RTE_BBDEV_DEV_FATAL_ERR) {
				if (accel_dev->device_reset_using_flr == DEVICE_RESET_USING_FLR)
					accel_dev->dev_status[i] = RTE_BBDEV_DEV_RESTART_REQ;
				else
					accel_dev->dev_status[i] = RTE_BBDEV_DEV_RECONFIG_REQ;
			} else
				accel_dev->dev_status[i] = RTE_BBDEV_DEV_CONFIGURED;
		} else
			accel_dev->dev_status[i] = status;
	}
}

int
update_reset_mode(void *dev, int mode)
{
	hw_device *accel_dev = (hw_device *)dev;

	LOG(DEBUG, "Change reset mode:");

	accel_dev->device_reset_using_flr = mode;

	LOG(DEBUG, "Done");

	return 0;
}

int
auto_reset_mode(void *dev, int mode)
{
	hw_device *accel_dev = (hw_device *)dev;

	LOG(DEBUG, "Auto reset mode change: ");

	accel_dev->auto_reconfig_on_fatal_error = mode;

	LOG(DEBUG, "Done");

	return 0;
}

void
clear_log_file(void *dev)
{
	char logFile[BB_ACC_LOG_FILE_LEN];
	hw_device *accel_dev = (hw_device *)dev;
	sprintf(logFile, "%s/pf_bb_cfg_%s.log", BB_ACC_DEFAULT_LOG_PATH,
			accel_dev->pci_address);
	LOG(INFO, "logFile = %s", logFile);
	bb_acc_reset_logFile(logFile);
}

void
exit_app_mode(void *dev)
{
	int r;
	char sysCmd[1024];
	memset(sysCmd, 0, sizeof(sysCmd));
	/* Send kill signal to pf_bb_config */
	r = system("pkill pf_bb_config");
	if (!r)
		LOG(INFO, "successfully kill pf_bb_config");
	else
		LOG(ERR, "pf_bb_config is not in running state");
}

int
acc100_reg_dump(hw_device *accel_dev,
		struct acc100_reg_dump_info *rd_db,
		int num_regs)
{
	int i = 0;
	int payload = 0;
	for (i = 0; i < num_regs; i++) {
		payload = reg_read(
				accel_dev->bar0Addr,
				rd_db[i].reg_offset);
		LOG(INFO, "%s\t\t, 0x%08x, 0x%08x", rd_db[i].reg_name,
				rd_db[i].reg_offset, payload);
	}
	return 0;
}

int
acc200_reg_dump(hw_device *accel_dev,
		struct acc200_reg_dump_info *rd_db,
		int num_regs)
{
	int i = 0;
	int payload = 0;
	for (i = 0; i < num_regs; i++) {
		payload = reg_read(
				accel_dev->bar0Addr,
				rd_db[i].reg_offset);
		LOG(INFO, "%s\t\t, 0x%08x, 0x%08x", rd_db[i].reg_name,
				rd_db[i].reg_offset, payload);
	}
	return 0;
}

int acc_reg_dump(void *dev, int devType)
{
	hw_device *device = (hw_device *)dev;
	int num;

	if (device->device_id != devType) {
		printf("ERR: Wrong device configured\n");
		return 0;
	}

	if (devType == ACC100_DEVICE_ID) {
		num = sizeof(acc100_rd_db) / sizeof(struct acc100_reg_dump_info);
		acc100_reg_dump(device, acc100_rd_db, num);
	} else if (devType == ACC200_DEVICE_ID) {
		num = sizeof(acc200_rd_db) / sizeof(struct acc200_reg_dump_info);
		acc200_reg_dump(device, acc200_rd_db, num);
	} else
		printf("ERR: Wrong Device !!!\n");

	return 0;
}

int acc_mem_read(void *dev, int rwFlag, int regAddr, int wPayload)
{
	hw_device *device = (hw_device *)dev;
	uint32_t payload = 0x0;
	LOG(DEBUG, "rwFlag = %d, regAddr = 0x%x, wPayload = 0x%x\n",
			rwFlag, regAddr, wPayload);
	switch (rwFlag) {
	default:
	case MM_READ_REG_READ:
		payload = reg_read(device->bar0Addr, regAddr);
		LOG(INFO, "Read 0x%08X 0x%08X\n", regAddr, payload);
		break;
	case MM_READ_REG_WRITE:
		LOG(INFO, "Write 0x%08X 0x%08X\n", regAddr, wPayload);
		reg_write(device->bar0Addr, regAddr, wPayload);
		break;
	}
	return 0;
}

void
print_all_stat32(hw_device *accel_dev, uint32_t address, int num, int reg_offset)
{
	char buf[512];
	size_t len;
	uint32_t vf_idx, payload;

	memset(buf, 0, 512);
	len = 0;

	for (vf_idx = 0; vf_idx < num; vf_idx++) {
		payload = reg_read(accel_dev->bar0Addr, address + reg_offset * vf_idx);
		len = print_stat32(buf, len, 1024, payload);
	}
	LOG(INFO, "%s", buf);
}

int
print_stat32(char *buf, int offset, int len, uint32_t stat)
{
	offset += snprintf(buf + offset, len - offset, "%u ", stat);
	return offset;
}

void acc_device_data(void *dev)
{
	hw_device *device = (hw_device *)dev;

	if (device->device_id == ACC100_DEVICE_ID)
		acc100_device_data(device);
	else if (device->device_id == ACC200_DEVICE_ID)
		acc200_device_data(device);
}

