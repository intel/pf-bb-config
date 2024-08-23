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

/* Protected address ranges. */
#define VRB1_QMGR_BA_START 0xA01000
#define VRB1_QMGR_BA_END   0xA02000
#define VRB1_ARAM_START    0x818000
#define VRB1_ARAM_END      0x820000
#define VRB1_QOS_A_START   0xB900C0
#define VRB1_QOS_A_END     0xB900E0
#define VRB1_QOS_B_START   0xBA00C0
#define VRB1_QOS_B_END     0xBA00E0
#define VRB1_QOS_C_START   0xBB00C0
#define VRB1_QOS_C_END     0xBB00E0
#define VRB2_QMGR_BA_START 0xA08000
#define VRB2_QMGR_BA_END   0xA0A000
#define VRB2_ARAM_START    0x818000
#define VRB2_ARAM_END      0x820000
#define VRB2_MLD_START     0xB5E000
#define VRB2_MLD_END       0xB60000

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
	if (accel_dev->ops.conf(accel_dev, accel_dev->bar0Addr, accel_dev->config_file,
			BB_ACC_RECFG)) {
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

/* Do both a cluster reset then a PFLR. */
int
bb_acc_cluster_reset_and_flr_reconfig(void *dev)
{
	hw_device *accel_dev = (hw_device *)dev;

	LOG(DEBUG, "Cluster reset and reconfig");

	if (accel_dev == NULL)
		return 0;

	/* Disable interrupts. */
	if ((accel_dev->ops.disable_intr) &&
			(accel_dev->ops.disable_intr(accel_dev))) {
		LOG(ERR, "Disable interrupts failed");
		return -1;
	}

	/* Cluster reset first. */
	if (accel_dev->ops.cluster_reset)
		accel_dev->ops.cluster_reset(accel_dev);
	else
		LOG(ERR, "Cluster reset is not supported");

	/* PF FLR. */
	if ((accel_dev->ops.flr) &&
			(accel_dev->ops.flr(accel_dev))) {
		LOG(ERR, "Device reset failed");
	}

	/* Re-init and configure the device. */
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
	char log_file[BB_ACC_LOG_FILE_LEN];
	hw_device *accel_dev = (hw_device *)dev;
	sprintf(log_file, "%s/pf_bb_cfg_%s.log", BB_ACC_DEFAULT_LOG_PATH,
			accel_dev->pci_address);
	LOG(DEBUG, "log_file = %s", log_file);
	bb_acc_reset_logFile(log_file, BB_ACC_LOG_MAIN);
}

void
clear_log_resp_file(void *dev)
{
	char log_file[BB_ACC_LOG_FILE_LEN];
	hw_device *accel_dev = (hw_device *)dev;
	sprintf(log_file, "%s/pf_bb_cfg_%s_response.log", BB_ACC_DEFAULT_LOG_PATH,
			accel_dev->pci_address);
	LOG(DEBUG, "logRespFile = %s", log_file);
	bb_acc_reset_logFile(log_file, BB_ACC_LOG_RESP);
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
		LOG_RESP(INFO, "%s\t\t, 0x%08x, 0x%08x", rd_db[i].reg_name,
				rd_db[i].reg_offset, payload);
	}
	return 0;
}

int
vrb1_reg_dump(hw_device *accel_dev,
		struct vrb1_reg_dump_info *rd_db,
		int num_regs)
{
	int i = 0;
	int payload = 0;
	for (i = 0; i < num_regs; i++) {
		payload = reg_read(
				accel_dev->bar0Addr,
				rd_db[i].reg_offset);
		LOG_RESP(INFO, "%s\t\t, 0x%08x, 0x%08x", rd_db[i].reg_name,
				rd_db[i].reg_offset, payload);
	}
	return 0;
}

int
vrb2_reg_dump(hw_device *accel_dev,
		struct vrb2_reg_dump_info *rd_db,
		int num_regs)
{
	int i = 0;
	int payload = 0;
	for (i = 0; i < num_regs; i++) {
		payload = reg_read(
				accel_dev->bar0Addr,
				rd_db[i].reg_offset);
		LOG_RESP(INFO, "%s\t\t, 0x%08x, 0x%08x", rd_db[i].reg_name,
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
	clear_log_resp_file(dev);

	if (devType == ACC100_DEVICE_ID) {
		num = sizeof(acc100_rd_db) / sizeof(struct acc100_reg_dump_info);
		acc100_reg_dump(device, acc100_rd_db, num);
	} else if (devType == VRB1_DEVICE_ID) {
		num = sizeof(vrb1_rd_db) / sizeof(struct vrb1_reg_dump_info);
		vrb1_reg_dump(device, vrb1_rd_db, num);
	} else if (devType == VRB2_DEVICE_ID) {
		num = sizeof(vrb2_rd_db) / sizeof(struct vrb2_reg_dump_info);
		vrb2_reg_dump(device, vrb2_rd_db, num);
	} else
		LOG_RESP(ERR, "Wrong Device for ref_dump!");

	LOG_RESP_END();
	return 0;
}

int acc_mem_read(void *dev, int rwFlag, int regAddr, int wPayload)
{
	hw_device *device = (hw_device *)dev;
	uint32_t payload = 0x0;
	LOG(DEBUG, "rwFlag = %d, regAddr = 0x%x, wPayload = 0x%x\n",
			rwFlag, regAddr, wPayload);
	clear_log_resp_file(dev);
	/* Protection specific to VRB registers. */
	if (device->device_id == VRB1_DEVICE_ID) {
		if ((regAddr >= VRB1_QMGR_BA_START) && (regAddr < VRB1_QMGR_BA_END)) {
			LOG_RESP(ERR, "Invalid Address for VRB1 0x%x", regAddr);
			LOG_RESP_END();
			return 0;
		}
		if ((regAddr >= VRB1_ARAM_START) && (regAddr < VRB1_ARAM_END)) {
			LOG_RESP(ERR, "Invalid ARAM Address for VRB1 0x%x", regAddr);
			LOG_RESP_END();
			return 0;
		}
		if ((regAddr >= VRB1_QOS_A_START) && (regAddr < VRB1_QOS_A_END)) {
			LOG_RESP(ERR, "Invalid QoS A Address for VRB1 0x%x", regAddr);
			LOG_RESP_END();
			return 0;
		}
		if ((regAddr >= VRB1_QOS_B_START) && (regAddr < VRB1_QOS_B_END)) {
			LOG_RESP(ERR, "Invalid QoS B Address for VRB1 0x%x", regAddr);
			LOG_RESP_END();
			return 0;
		}
		if ((regAddr >= VRB1_QOS_C_START) && (regAddr < VRB1_QOS_C_END)) {
			LOG_RESP(ERR, "Invalid QoS C Address for VRB1 0x%x", regAddr);
			LOG_RESP_END();
			return 0;
		}
	}
	if (device->device_id == VRB2_DEVICE_ID) {
		if ((regAddr >= VRB2_QMGR_BA_START) && (regAddr < VRB2_QMGR_BA_END)) {
			LOG_RESP(ERR, "Invalid Address for VRB2 0x%x", regAddr);
			LOG_RESP_END();
			return 0;
		}
		if ((regAddr >= VRB2_ARAM_START) && (regAddr < VRB2_ARAM_END)) {
			LOG_RESP(ERR, "Invalid ARAM Address for VRB2 0x%x", regAddr);
			LOG_RESP_END();
			return 0;
		}
		if ((regAddr >= VRB2_MLD_START) && (regAddr < VRB2_MLD_END)) {
			LOG_RESP(ERR, "Invalid MLD Address for VRB2 0x%x", regAddr);
			LOG_RESP_END();
			return 0;
		}
	}

	/* Prevent 4B read outside of PF BAR. */
	if ((regAddr + 4) > device->bar_size) {
		LOG_RESP(ERR, "Invalid address range for PF BAR 0x%x", regAddr);
		LOG_RESP_END();
		return 0;
	}

	switch (rwFlag) {
	default:
	case MM_READ_REG_READ:
		payload = reg_read(device->bar0Addr, regAddr);
		LOG_RESP(INFO, "Read 0x%08X 0x%08X\n", regAddr, payload);
		break;
	case MM_READ_REG_WRITE:
		LOG_RESP(INFO, "Write 0x%08X 0x%08X\n", regAddr, wPayload);
		reg_write(device->bar0Addr, regAddr, wPayload);
		break;
	}
	LOG_RESP_END();
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
	LOG_RESP(INFO, "%s", buf);
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
	clear_log_resp_file(dev);

	if (device->device_id == ACC100_DEVICE_ID)
		acc100_device_data(device);
	else if (device->device_id == VRB1_DEVICE_ID)
		vrb1_device_data(device);
	else if (device->device_id == VRB2_DEVICE_ID)
		vrb2_device_data(device);
	LOG_RESP_END();
}

