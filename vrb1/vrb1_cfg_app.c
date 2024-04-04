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
#include "vrb1_cfg_app.h"
#include "vrb1_fft_lut.h"
#include "vrb1_pf_enum.h"
#include "bb_acc.h"
#include "bb_acc_log.h"

static vrb1_ir_int_type_info histatus[32] = {
	{"5G_EXTRA_COMPLETION_RECVD",  true, true},  /* 0 */
	{"5G_COMPLETION_READ_TIMEOUT", true, true},  /* 1 */
	{"DMA_HW_ERROR_STATUS",        true, true},  /* 2 */
	{"FEC_SLICE_ERROR_STATUS",     true, true},  /* 3 */
	{"PROCESS_TO_STATUS",          true, true},  /* 4 */
	{"UNUSED", false, false},
	{"UNUSED", false, false},
	{"UNUSED", false, false},
	{"UNUSED", false, false},
	{"UNUSED", false, false},
	{"UNUSED", false, false},
	{"UNUSED", false, false},
	{"UNUSED", false, false},
	{"UNUSED", false, false},
	{"UNUSED", false, false},
	{"UNUSED", false, false},
	{"QMGR_ACC_TO_STATUS",          true, true},      /* 16 */
	{"ARAM_PROCESS_TO_STATUS",      true, true},      /* 17 */
	{"QMGR_PROCESS_TO_STATUS",      true, true},      /* 18 */
	{"MSI_FIFO_OF_STATUS",          true, false},     /* 19 */
	{"UNUSED", false, false},                         /* 20 */
	{"UNUSED", false, false},                         /* 21 */
	{"ARAM_ACCESS_ERR_STATUS",      true, true},      /* 22 */
	{"ARAM_ECC_ERR_STATUS",         true, false},     /* 23 */
	{"PARITY_FATAL_STATUS",         true, true},      /* 24 */
	{"PARITY_NONFATAL_STATUS",      true, false},     /* 25 */
	{"APB_TO_STATUS",               true, true},      /* 26 */
	{"INTERRUPT_FIFO_OF_STATUS",    true, false},     /* 27 */
	{"IOSF_BRIDGE_ERR_STATUS",      true, true},      /* 28 */
	{"HI_1B_ECC",                   true, false},     /* 29 */
	{"HI_MB_ECC",                   true, true},      /* 30 */
	{"UNUSED",                      false, false},
};

static vrb1_ir_int_type_info extError[32] = {
	{"DMADL_PAR_ERR_STATUS", true },       /* 0  */
	{"DMAUL_PAR_ERR_STATUS", true },       /* 1  */
	{"QMGR_PAR_ERR_STATUS", true },        /* 2  */
	{"ARAM_PAR_ERR_STATUS", true },        /* 3  */
	{"PAR_ERR_STATUS_5G", true },          /* 4  */
	{"UNUSED", false },                    /* 5  */
	{"UNUSED", false },                    /* 6  */
	{"UNUSED", false },                    /* 7  */
	{"UNUSED", false },                    /* 8  */
	{"UNUSED", false },                    /* 9  */
	{"UNUSED", false },                    /* 10 */
	{"UNUSED", false },                    /* 11 */
	{"UNUSED", false },                    /* 12 */
	{"UNUSED", false },                    /* 13 */
	{"UNUSED", false },                    /* 14 */
	{"UNUSED", false },                    /* 15 */
	{"UNUSED", false },                    /* 16 */
	{"UNUSED", false },                    /* 17 */
	{"DMA_HW_ERROR", true},                /* 18 */
	{"READ_TIMEOUT_STATUS_5G", true},      /* 19 */
	{"EXTRA_READ_STATUS_5G", true},        /* 20 */
	{"UNUSED", false},                     /* 21 */
	{"UNUSED", false},                     /* 22 */
	{"ARAM_ACCESS_ERR_STATUS", true},      /* 23 */
	{"UNUSED", false},                     /* 24 */
	{"UNUSED", false},                     /* 25 */
	{"QMGR_HI_FIFO_OFLOW_STATUS", false},  /* 26 */
	{"QMGR_ERROR_DETECTED_STATUS", true},  /* 27 */
	{"UNUSED", false},
	{"UNUSED", false},
	{"UNUSED", false},
	{"UNUSED", false},
};

#define PF_TO_VF_DBELL_REG_OFFSET 0x100

static void
vrb1_reg_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
	usleep(1);
}

static void
vrb1_reg_fast_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
}

static uint32_t
vrb1_reg_read(uint8_t *mmio_base, uint32_t offset)
{

	void *reg_addr = mmio_base + offset;
	uint32_t ret = *((volatile uint32_t *)(reg_addr));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	ret = __bswap_32(ret);
#endif
	return ret;
}

/* Manage PF to VF communications */
static void
vrb1_pfvf(void *dev, unsigned int vf_index, unsigned int payload)
{
	hw_device *accel_dev = (hw_device *)dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;
	unsigned int window_index;

	LOG(DEBUG, "Doorbell vf2pf %d", payload);

	if (payload == REQ_DEV_STATUS) {
		vrb1_reg_fast_write(bar0addr, HWPfHiPfToVfDbellVf +
					BB_ACC_PF_TO_VF_DBELL_REG_OFFSET * vf_index,
					accel_dev->dev_status[vf_index]);
		if (accel_dev->prev_status[vf_index] != accel_dev->dev_status[vf_index]) {
			LOG(INFO, "Report Device Status VF: %u Status %lu %s",
				vf_index, accel_dev->dev_status[vf_index],
				bb_acc_device_status_str(accel_dev->dev_status[vf_index]));
			accel_dev->prev_status[vf_index] = accel_dev->dev_status[vf_index];
		}
		/* Clear back when read once */
		if (accel_dev->dev_status[vf_index] == RTE_BBDEV_DEV_RESTART_REQ ||
				accel_dev->dev_status[vf_index] == RTE_BBDEV_DEV_RECONFIG_REQ)
			accel_dev->dev_status[vf_index] = RTE_BBDEV_DEV_CONFIGURED;
	}
	/* Reports a version number based on the used FFT LUT binary file. */
	if (payload == REQ_DEV_LUT_VER) {
		vrb1_reg_fast_write(bar0addr, HWPfHiPfToVfDbellVf +
				vf_index * BB_ACC_PF_TO_VF_DBELL_REG_OFFSET,
				(uint32_t)accel_dev->fft_version_md5sum);
	}
	/* Reports Window sizes based on the used FFT LUT binary file. */
	if ((payload & REQ_DEV_MASK) == REQ_DEV_FFT_WIN_SIZE) {
		window_index = payload >> 16;
		vrb1_reg_fast_write(bar0addr, HWPfHiPfToVfDbellVf
				+ vf_index * BB_ACC_PF_TO_VF_DBELL_REG_OFFSET,
				(uint32_t)accel_dev->fft_win_size[window_index]);
	}
	if ((payload & REQ_DEV_MASK) == REQ_DEV_FFT_WIN_START) {
		window_index = payload >> 16;
		vrb1_reg_fast_write(bar0addr, HWPfHiPfToVfDbellVf
				+ vf_index * BB_ACC_PF_TO_VF_DBELL_REG_OFFSET,
				(uint32_t)accel_dev->fft_win_start[window_index]);
	}
	if (payload == REQ_DEV_NEW && accel_dev->dev_status[vf_index] == RTE_BBDEV_DEV_CONFIGURED) {
		accel_dev->dev_status[vf_index] = RTE_BBDEV_DEV_ACTIVE;
		LOG(INFO, "VF Device %u becomes active", vf_index);
	}

	LOG(DEBUG, "Done");
}

static bool
vrb1_device_error(void *dev, int int_src)
{
	uint32_t err_status;
	uint16_t i;
	hw_device *accel_dev = (hw_device *)dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;
	bool fatal_error = false;

	err_status = vrb1_reg_read(bar0addr, HWPfHiErrStatusReg);
	for (i = 0; i < 32; i++) {
		if (((err_status >> i) & 0x1) == 0x1) {
			if (histatus[i].fatal)
				fatal_error = true;
			LOG(ERR, "HI Device Error: %d %s", i, histatus[i].name);
		}
	}

	err_status = vrb1_reg_read(bar0addr, HWPfDmaAxiStatus);
	if ((err_status & 0x1) == 0x1) {
		fatal_error = true;
		LOG(ERR, "Stop AXI asserted");
	}
	err_status = vrb1_reg_read(bar0addr, HWPfHiIosf2axiErrLogReg);
	for (i = 0; i < 32; i++)
		if (((err_status >> i) & 0x1) == 0x1)
			LOG(ERR, "Ext Device Error: %d %s", i, extError[i].name);

	/* Clear error injection */
	if (fatal_error) {
		vrb1_reg_fast_write(bar0addr, HWPfHiErrInjectReg, 0);
		vrb1_reg_fast_write(bar0addr, HWPfAramControlStatus, VRB1_ARAM_CONTROL);
		vrb1_reg_fast_write(bar0addr, HWPfDmaCmplTmOutCnt, 0xFFFFFFFF);
		vrb1_reg_fast_write(bar0addr, HWPfDmaProcTmOutCnt, VRB1_PROC_TIMEOUT);
		vrb1_reg_fast_write(bar0addr, HWPfQmgrAramWatchdogCount, 0xFFFFFFFF);
		vrb1_reg_fast_write(bar0addr, HWPfQmgrAxiWatchdogCount, 0xFFFFFFFF);
		vrb1_reg_fast_write(bar0addr, HWPfQmgrProcessWatchdogCount, 0xFFFFFFFF);
	}
	if (fatal_error && (accel_dev->dev_status[0] != RTE_BBDEV_DEV_FATAL_ERR)) {
		LOG(ERR, "Fatal error");
		/* For these conditions a device fatal error was hit */
		bb_acc_set_all_device_status(accel_dev, RTE_BBDEV_DEV_FATAL_ERR);
		/*
		 * When fatal error is detected, the device may be reset or not according to
		 * the related device attributes.
		 */
		if (accel_dev->auto_reconfig_on_fatal_error) {
			/* Clear back the status register */
			vrb1_reg_fast_write(bar0addr, HWPfHiIosf2axiErrLogReg, 0xFFFFFFFF);
			/* reset and reconfigure the device */
			if (accel_dev->device_reset_using_flr == DEVICE_RESET_USING_FLR) {
				LOG(INFO, "FLR and reconfig");
				if (bb_acc_cluster_reset_and_flr_reconfig(accel_dev))
					LOG(ERR, "FLR and reconfig failed");
			} else {
				LOG(INFO, "Cluster reset and reconfig");
				if (bb_acc_cluster_reset_and_reconfig(accel_dev))
					LOG(ERR, "Cluster reset and reconfig failed");
			}
		}
	}
	return fatal_error;
}

int
vrb1_irq_handler(void *dev)
{
	hw_device *accel_pci_dev = (hw_device *)dev;
	uint8_t *bar0addr = accel_pci_dev->bar0Addr;
	uint32_t vf_index, status;
	/* No infoRing data available */

	/* Check first VF2PF comms */
	for (vf_index = 0; vf_index < VRB1_NUM_VFS; vf_index++) {
		status = vrb1_reg_read(bar0addr, HWPfHiVfToPfDbellVf +
				BB_ACC_PF_TO_VF_DBELL_REG_OFFSET * vf_index);
		if (status != 0) {
			vrb1_pfvf(dev, vf_index, status);
			LOG(DEBUG, "door bell %d %d", vf_index, status);
		}
	}
	/* Check for device error */
	vrb1_device_error(dev, VRB1_ITR_UNDEFINED);
	return 0;
}

/**
 * Retrieve VRB1 device configuration from file. The file's content must keep
 * the following structure:
 *
 * [MODE]
 * pf_mode_en =
 *
 * [VFBUNDLES]
 * num_vf_bundles =
 *
 * [MAXQSIZE]
 * max_queue_size =
 *
 * [QUL]
 * num_qgroups =
 * aq_depth_log2 =
 *
 * [QDL]
 * num_qgroups =
 * num_aqs_per_groups =
 * aq_depth_log2 =
 *
 * [ARBUL0]
 * round_robin_weight =
 * gbr_threshold1 =
 * gbr_threshold2 =
 *
 * [ARBUL1]
 * round_robin_weight =
 * gbr_threshold1 =
 * gbr_threshold2 =
 *
 * [ARBDL0]
 * round_robin_weight =
 * gbr_threshold1 =
 * gbr_threshold2 =
 *
 * [ARBDL1]
 * round_robin_weight =
 * gbr_threshold1 =
 * gbr_threshold2 =
 *
 * @param file_name
 *   The location of the configuration file.
 * @param vrb1_conf
 *   Pointer to structure that will hold VRB1 configuration
 *
 * @return
 *   Zero on success, negative value on failure.
statir void
vrb1_reg_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
	usleep(1000);
}

static void
vrb1_reg_fast_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
}

static uint32_t
vrb1_reg_read(uint8_t *mmio_base, uint32_t offset)
{

	void *reg_addr = mmio_base + offset;
	uint32_t ret = *((volatile uint32_t *)(reg_addr));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	ret = __bswap_32(ret);
#endif
	return ret;
}

 */
static int
vrb1_read_config_file(const char *arg_cfg_filename,
		struct vrb1_conf *vrb1_conf)
{
	bool unsafe_path = cfg_file_check_path_safety(arg_cfg_filename);
	if (unsafe_path == true) {
		printf("error, config file path \"%s\" is not safe",
				arg_cfg_filename);
		return -1;
	} else
		return vrb1_parse_conf_file(arg_cfg_filename, vrb1_conf);
}

enum {UL_4G = 0, UL_5G, DL_4G, DL_5G, FFT, NUM_ACC};

/* Return the que topology for a Queue Group Index */
void
vrb1_qtopFromAcc(struct q_topology_t **qtop, int acc_enum,
		struct vrb1_conf *vrb1_conf)
{
	struct q_topology_t *p_qtop;
	p_qtop = NULL;
	switch (acc_enum) {
	case UL_4G:
		p_qtop = &(vrb1_conf->q_ul_4g);
		break;
	case UL_5G:
		p_qtop = &(vrb1_conf->q_ul_5g);
		break;
	case DL_4G:
		p_qtop = &(vrb1_conf->q_dl_4g);
		break;
	case DL_5G:
		p_qtop = &(vrb1_conf->q_dl_5g);
		break;
	case FFT:
		p_qtop = &(vrb1_conf->q_fft);
		break;
	default:
		/* NOTREACHED */
		printf("Unexpected error evaluating vrb1_qtopFromAcc %d",
				acc_enum);
		break;
	}
	*qtop = p_qtop;
}

/* Check LUT content for FFT windows. */
void
vrb_fft_win_check(int16_t *gTDWinCoff, hw_device *accel_pci_dev)
{
	int16_t i, offset_p = 0, offset_i, fft_sz, lut_sz, wrap1, wrap2, win_start, win_size, win;

	for (fft_sz = 32; fft_sz <= 2048; fft_sz *= 2) {
		/*
		 * Manage FFT wrap for negative and positive halves.
		 * Each table width is fft_sz / 2.
		 */
		lut_sz = fft_sz / 2;
		wrap1 = lut_sz / 2 * 3;
		wrap2 = lut_sz / 2;
		for (win = 0;  win < BB_ACC_MAX_WIN; win++) {
			offset_i = BB_ACC_MAX_WIN * offset_p + win * lut_sz;
			win_size = 0;
			win_start = 0;
			/* Check non null value in LUT for size and start of window shape. */
			for (i = 0; i < lut_sz; i++) {
				if (gTDWinCoff[offset_i + ((wrap1 - i) % lut_sz)] > 0) {
					win_size++;
					win_start = wrap2 - i;
				}
			}
			win_size = MAX(1, win_size);
			if (fft_sz == 2048) {
				accel_pci_dev->fft_win_size[win] = win_size;
				accel_pci_dev->fft_win_start[win] = win_start;
			}
			if ((win == 0) && (fft_sz == 2048))
				LOG(INFO, "  FFT Size %d Window %d Size %d Start %d",
						fft_sz, win, win_size, win_start);
		}
		offset_p += lut_sz;
	}
}

void
vrb_fft_lut_md5sum(const char *lut_filename, hw_device *accel_pci_dev)
{
	uint16_t ver_md;
	char md_prefix[8];
	char *cmd;
	FILE *fp;

	uint16_t size_cmd = snprintf(NULL, 0, "%s", lut_filename) + 8;
	cmd = malloc(size_cmd);
	sprintf(cmd, "md5sum %s", lut_filename);
	fp = popen(cmd, "r");
	free(cmd);
	if (fp == NULL) {
		LOG(ERR, "Unable to open FFT LUT %s", lut_filename);
		return;
	}
	if (fgets(md_prefix, 5, fp) == NULL) {
		LOG(ERR, "Unable to access FFT LUT %s", lut_filename);
		pclose(fp);
		return;
	}
	ver_md = (int)strtol(md_prefix, NULL, 16);
	LOG(INFO, "  FFT Version Number %X", ver_md);
	pclose(fp);
	accel_pci_dev->fft_version_md5sum = ver_md;
}

/* Configure or reconfigure a given FFT engine. */
static int
vrb1_fft_reconfig(uint8_t *d, bool lut_programming, hw_device *accel_pci_dev)
{
	int16_t gTDWinCoff[VRB1_LUT_SIZE], ret, offset, i, pageIdx;
	const char *lut_filename;
	FILE *fp;
	bool unsafe_path;

	/* Default FFT configuration. */
	vrb1_reg_fast_write(d, HWPfFftConfig0, VRB1_FFT_CFG_0);

	if (!lut_programming)
		return 0;

	/* Load the FFT windows LUT. */
	lut_filename = accel_pci_dev->fft_lut_filename;
	unsafe_path = cfg_file_check_path_safety(lut_filename);
	if (unsafe_path == true) {
		LOG(ERR, "error, FFT LUT file path \"%s\" is not safe", lut_filename);
		return -ENOENT;
	}

	fp = fopen(lut_filename, "rb");
	if (fp == NULL) {
		LOG(ERR, "  Error reading from FFT LUT %s", lut_filename);
		return -ENOENT;
	}
	ret = fread((char *)gTDWinCoff, VRB1_LUT_SIZE * 2, 1, fp);
	if (ret <= 0) {
		LOG(ERR, "  Error reading from FFT LUT %s", lut_filename);
		fclose(fp);
		return -ENOENT;
	}
	LOG(INFO, "  FFT Window coeffs preloading from %s", lut_filename);

	offset = VRB1_FFT_FIRST_OFFSET;
	pageIdx = 0;
	vrb1_reg_fast_write(d, HWPfFftRamPageAccess, VRB1_FFT_RAM_EN);
	for (i = 0; i < VRB1_LUT_SIZE; i++) {
		vrb1_reg_fast_write(d, HWPfFftRamOff + offset *	VRB1_BYTES_IN_WORD, gTDWinCoff[i]);
		offset++;
		if ((offset % VRB1_FFT_PAGE_SIZE) == 0) {
			offset = 0;
			pageIdx++;
			fflush(stdout);
			vrb1_reg_fast_write(d, HWPfFftRamPageAccess, VRB1_FFT_RAM_EN + pageIdx);
		}
	}

	vrb1_reg_fast_write(d, HWPfFftRamPageAccess, VRB1_FFT_RAM_DIS);
	fclose(fp);

	vrb_fft_win_check(gTDWinCoff, accel_pci_dev);

	vrb_fft_lut_md5sum(lut_filename, accel_pci_dev);

	return 0;
}

/* Return the accelerator enum for a Queue Group Index */
int
vrb1_acc_from_qgid(int qg_idx, struct vrb1_conf *vrb1_conf)
{
	int acc_qg[VRB1_NUM_QGRPS];
	int num_qgroups_per_fn[NUM_ACC];
	int acc, qgid, qg_index = 0;

	for (qgid = 0; qgid < VRB1_NUM_QGRPS; qgid++)
		acc_qg[qgid] = 0;
	num_qgroups_per_fn[UL_4G] = vrb1_conf->q_ul_4g.num_qgroups;
	num_qgroups_per_fn[UL_5G] = vrb1_conf->q_ul_5g.num_qgroups;
	num_qgroups_per_fn[DL_4G] = vrb1_conf->q_dl_4g.num_qgroups;
	num_qgroups_per_fn[DL_5G] = vrb1_conf->q_dl_5g.num_qgroups;
	num_qgroups_per_fn[FFT] = vrb1_conf->q_fft.num_qgroups;
	for (acc = UL_4G;  acc < NUM_ACC; acc++)
		for (qgid = 0; qgid < num_qgroups_per_fn[acc]; qgid++)
			acc_qg[qg_index++] = acc;
	acc = acc_qg[qg_idx];
	return acc;
}

/* Return the que topology for a Queue Group Index */
void
vrb1_qtop_from_acc(struct q_topology_t **qtop, int acc_enum,
		     struct vrb1_conf *vrb1_conf)
{
	struct q_topology_t *p_qtop;

	p_qtop = NULL;
	switch (acc_enum) {
	case UL_4G:
		p_qtop = &vrb1_conf->q_ul_4g;
		break;
	case UL_5G:
		p_qtop = &vrb1_conf->q_ul_5g;
		break;
	case DL_4G:
		p_qtop = &vrb1_conf->q_dl_4g;
		break;
	case DL_5G:
		p_qtop = &vrb1_conf->q_dl_5g;
		break;
	case FFT:
		p_qtop = &vrb1_conf->q_fft;
		break;
	default:
		/* NOTREACHED */
		printf("Unexpected error evaluating queue topology %d", acc_enum);
		break;
	}
	*qtop = p_qtop;
}

/* Return the AQ depth for a Queue Group Index */
int
vrb1_aq_depth(int qg_idx, struct vrb1_conf *vrb1_conf)
{
	struct q_topology_t *q_top = NULL;
	int acc_enum = vrb1_acc_from_qgid(qg_idx, vrb1_conf);

	vrb1_qtop_from_acc(&q_top, acc_enum, vrb1_conf);
	if (q_top == NULL)
		return 1;
	return MAX(1, q_top->aq_depth_log2);
}

/* Return the AQ depth for a Queue Group Index */
int
vrb1_aq_num(int qg_idx, struct vrb1_conf *vrb1_conf)
{
	struct q_topology_t *q_top = NULL;
	int acc_enum = vrb1_acc_from_qgid(qg_idx, vrb1_conf);

	vrb1_qtop_from_acc(&q_top, acc_enum, vrb1_conf);
	if (!q_top)
		return 0;
	return q_top->num_aqs_per_groups;
}

/* Generate Qmgr templates mask */
uint32_t vrb1_genmask(int num_qqs_acc, int num_qgs)
{
	if (num_qgs < 1)
		return 0;
	else
		return GENMASK(num_qgs + num_qqs_acc - 1, num_qqs_acc);
}

/* Register offset for QoS Registers */
static int
vrb1_qos_offset(int vf_idx, int xl)
{
	int register_offset = HWPfQosmonBCntrlReg - HWPfQosmonACntrlReg;
	return VF_OFFSET_QOS * vf_idx + xl * register_offset;
}

/* Register offset for QoS Registers */
static int
vrb1_qos_action(int idx, int xl)
{
	int register_offset = HWPfQosmonBCntrlReg - HWPfQosmonACntrlReg;
	return 4 * idx + xl * register_offset;
}

static int
vrb1_write_config(void *dev, void *mapaddr, struct vrb1_conf *vrb1_conf, const bool first_cfg)
{
	uint32_t value, address, status;
	int qg_idx, template_idx, vf_idx, acc, i, xl;
	/* QMGR_ARAM - memory internal to VRB1 */
	uint32_t aram_address = 0;
	int rlim = 0;
	int alen = 1;
	int timestamp_en = 0;
	int num_qgs, ret;
	int num_qqs_acc = 0;
	int num_engines = 0;
	int total_qgs = 0;
	int qman_func_id[8] = {0, 2, 1, 3, 4, 0, 0, 0};
	uint8_t *d = mapaddr;
	hw_device *accel_dev = (hw_device *)dev;

	/* Check we are already out of PG */
	status = vrb1_reg_read(d, HWPfHiSectionPowerGatingAck);
	if (status > 0) {
		if (status != VRB1_PG_MASK_0) {
			LOG(ERR, "Unexpected status %x %x", status, VRB1_PG_MASK_0);
			return -ENODEV;
		}
		/* Clock gate sections that will be un-PG */
		vrb1_reg_write(d, HWPfHiClkGateHystReg, VRB1_CLK_DIS);
		/* Un-PG required sections */
		vrb1_reg_write(d, HWPfHiSectionPowerGatingReq, VRB1_PG_MASK_1);
		status = vrb1_reg_read(d, HWPfHiSectionPowerGatingAck);
		if (status != VRB1_PG_MASK_1) {
			LOG(ERR, "Unexpected status %x %x", status, VRB1_PG_MASK_1);
			return -ENODEV;
		}
		vrb1_reg_write(d, HWPfHiSectionPowerGatingReq, VRB1_PG_MASK_2);
		status = vrb1_reg_read(d, HWPfHiSectionPowerGatingAck);
		if (status != VRB1_PG_MASK_2) {
			LOG(ERR, "Unexpected status %x %x", status, VRB1_PG_MASK_2);
			return -ENODEV;
		}
		vrb1_reg_write(d, HWPfHiSectionPowerGatingReq, VRB1_PG_MASK_3);
		status = vrb1_reg_read(d, HWPfHiSectionPowerGatingAck);
		if (status != VRB1_PG_MASK_3) {
			LOG(ERR, "Unexpected status %x %x", status, VRB1_PG_MASK_3);
			return -ENODEV;
		}

		/* Explicitly reset QoS and QMGR SRAM to avoid parity error false alarm. */
		for (xl = 0; xl < VRB1_NUM_QOS; xl++) {
			for (vf_idx = 0; vf_idx < VRB1_NUM_VFS; vf_idx++) {
				address = HWPfQosmonARemThres1Vf + vrb1_qos_offset(vf_idx, xl);
				vrb1_reg_fast_write(d, address, 0);
				address = HWPfQosmonAThres2Vf + vrb1_qos_offset(vf_idx, xl);
				vrb1_reg_fast_write(d, address, 0);
				address = HWPfQosmonAWeiFracVf + vrb1_qos_offset(vf_idx, xl);
				vrb1_reg_fast_write(d, address, 0);
				address = HWPfQosmonARrWeiVf + vrb1_qos_offset(vf_idx, xl);
				vrb1_reg_fast_write(d, address, 0);
			}
			for (i = 0; i < 8; i++) {
				address = HWPfQosmonAXaction + vrb1_qos_action(i, xl);
				vrb1_reg_fast_write(d, address, 0);
			}
		}
		for (i = 0; i < VRB1_QM_BA_SIZE; i++) {
			address = HWPfQmgrVfBaseAddr + i * sizeof(uint32_t);
			vrb1_reg_fast_write(d, address, 0);
		}
	}

	/* Enable clocks for all sections */
	vrb1_reg_write(d, HWPfHiClkGateHystReg, VRB1_CLK_EN);

	/* Explicitly releasing AXI as this may be stopped after PF FLR/BME */
	address = HWPfDmaAxiControl;
	value = 1;
	vrb1_reg_fast_write(d, address, value);

	/* Set the fabric mode */
	address = HWPfFabricM2iBufferReg;
	value = VRB1_FABRIC_MODE;
	vrb1_reg_fast_write(d, address, value);

	/* Set default descriptor signature */
	address = HWPfDmaDescriptorSignatuture;
	value = 0;
	vrb1_reg_fast_write(d, address, value);

	/* Enable the Error Detection in DMA */
	value = VRB1_CFG_DMA_ERROR;
	address = HWPfDmaErrorDetectionEn;
	vrb1_reg_fast_write(d, address, value);

	/* AXI Cache configuration */
	value = VRB1_CFG_AXI_CACHE;
	address = HWPfDmaAxcacheReg;
	vrb1_reg_fast_write(d, address, value);

	/* AXI Response configuration */
	vrb1_reg_fast_write(d, HWPfDmaCfgRrespBresp, 0x0);

	/* Default DMA Configuration (Qmgr Enabled) */
	address = HWPfDmaConfig0Reg;
	value = 0;
	vrb1_reg_fast_write(d, address, value);
	address = HWPfDmaQmanen;
	value = 0;
	vrb1_reg_fast_write(d, address, value);

	/* Default RLIM/ALEN configuration */
	rlim = 0;
	alen = 1;
	timestamp_en = 0;
	address = HWPfDmaConfig1Reg;
	value = (1 << 31) + (rlim << 8) + (timestamp_en << 6) + alen;
	vrb1_reg_fast_write(d, address, value);

	/* Configure the FFT ECC Mask */
	vrb1_reg_fast_write(d, HwPfFftParityMaskReg0, 1 << 31);
	vrb1_reg_fast_write(d, HwPfFftParityMaskReg1, 1 << 31);

	/* Configure DMA Qmanager addresses */
	address = HWPfDmaQmgrAddrReg;
	value = HWPfQmgrEgressQueuesTemplate;
	vrb1_reg_fast_write(d, address, value);

	/* Enable time out counters */
	vrb1_reg_fast_write(d, HWPfDmaCmplTmOutCnt, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfDmaProcTmOutCnt, VRB1_PROC_TIMEOUT);
	vrb1_reg_fast_write(d, HWPfDmaConfigPtoutOutEn, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfDmaStatusToutProcess, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfDmaConfigCtoutOutDataEn, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfDmaConfigCtoutOutDescrEn, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfDmaConfigUnexpComplDataEn, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfDmaConfigUnexpComplDescrEn, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfDmaConfigPtoutOutEn, 0xFFFFFFFF);

	vrb1_reg_fast_write(d, HWPfQmgrAramWatchdogCount, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfQmgrAramWatchdogCounterEn, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfQmgrAxiWatchdogCount, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfQmgrAxiWatchdogCounterEn, 0xFFFFFFFF);
	for (address = 0; address < 8; address++)
		vrb1_reg_fast_write(d, HWPfQmgrProcessWatchdogCounterFunc +
				sizeof(uint32_t) * address, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfQmgrProcessWatchdogCount, 0xFFFFFFFF);
	vrb1_reg_fast_write(d, HWPfQmgrProcessWatchdogCounterEn, 0xFF);

	/* ===== Qmgr Configuration ===== */
	/* Configuration of the AQueue Depth QMGR_GRP_0_DEPTH_LOG2 for UL */
	total_qgs = vrb1_conf->q_ul_4g.num_qgroups +
		vrb1_conf->q_ul_5g.num_qgroups +
		vrb1_conf->q_dl_4g.num_qgroups +
		vrb1_conf->q_dl_5g.num_qgroups +
		vrb1_conf->q_fft.num_qgroups;

	LOG(INFO, "Queue Groups UL4G %d DL4G %d UL5G %d DL5G %d FFT %d",
			vrb1_conf->q_ul_4g.num_qgroups, vrb1_conf->q_dl_4g.num_qgroups,
			vrb1_conf->q_ul_5g.num_qgroups, vrb1_conf->q_dl_5g.num_qgroups,
			vrb1_conf->q_fft.num_qgroups);

	for (qg_idx = 0; qg_idx < total_qgs; qg_idx++) {
		address = HWPfQmgrDepthLog2Grp + sizeof(uint32_t) * qg_idx;
		value = vrb1_aq_depth(qg_idx, vrb1_conf);
		vrb1_reg_fast_write(d, address, value);
		address = HWPfQmgrTholdGrp + sizeof(uint32_t) * qg_idx;
		value = (1 << 16) + (1 << (vrb1_aq_depth(qg_idx, vrb1_conf) - 1));
		vrb1_reg_fast_write(d, address, value);
		address = HWPfQmgrArbQDepthGrp + sizeof(uint32_t) * qg_idx;
		vrb1_reg_fast_write(d, address, VRB1_ARB_QDEPTH);
	}

	/* Template Priority in incremental order */
	for (template_idx = 0; template_idx < VRB1_NUM_TMPL; template_idx++) {
		address = HWPfQmgrGrpTmplateReg0Indx + sizeof(uint32_t) * template_idx;
		value = VRB1_TMPL_PRI_0;
		vrb1_reg_fast_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg1Indx + sizeof(uint32_t) * template_idx;
		value = VRB1_TMPL_PRI_1;
		vrb1_reg_fast_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg2Indx + sizeof(uint32_t) * template_idx;
		value = VRB1_TMPL_PRI_2;
		vrb1_reg_fast_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg3Indx + sizeof(uint32_t) * template_idx;
		value = VRB1_TMPL_PRI_3;
		vrb1_reg_fast_write(d, address, value);
	}

	address = HWPfQmgrGrpPriority;
	value = VRB1_CFG_QMGR_HI_P;
	vrb1_reg_fast_write(d, address, value);

	/* Template Configuration */
	for (template_idx = 0; template_idx < VRB1_NUM_TMPL; template_idx++) {
		value = 0;
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		vrb1_reg_fast_write(d, address, value);
	}
	/* 4GUL */
	num_qgs = vrb1_conf->q_ul_4g.num_qgroups;
	num_qqs_acc = 0;
	value = vrb1_genmask(num_qqs_acc, num_qgs);
	for (template_idx = VRB1_SIG_UL_4G; template_idx <= VRB1_SIG_UL_4G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		vrb1_reg_fast_write(d, address, value);
	}
	/* 5GUL */
	num_qqs_acc += num_qgs;
	num_qgs = vrb1_conf->q_ul_5g.num_qgroups;
	num_engines = 0;
	value = vrb1_genmask(num_qqs_acc, num_qgs);
	for (template_idx = VRB1_SIG_UL_5G; template_idx <= VRB1_SIG_UL_5G_LAST;
			template_idx++) {
		/* Check engine power-on status */
		address = HWPfFecUl5gIbDebugReg + VRB1_ENGINE_OFFSET * template_idx;
		status = (vrb1_reg_read(d, address) >> 4) & 0x7;
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		if (status == 1) {
			vrb1_reg_fast_write(d, address, value);
			num_engines++;
		} else {
			vrb1_reg_fast_write(d, address, 0);
		}
	}
	LOG(DEBUG, "Number of 5GUL engines %d", num_engines);
	/* 4GDL */
	num_qqs_acc += num_qgs;
	num_qgs = vrb1_conf->q_dl_4g.num_qgroups;

	value = vrb1_genmask(num_qqs_acc, num_qgs);
	for (template_idx = VRB1_SIG_DL_4G; template_idx <= VRB1_SIG_DL_4G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		vrb1_reg_fast_write(d, address, value);
	}
	/* 5GDL */
	num_qqs_acc += num_qgs;
	num_qgs = vrb1_conf->q_dl_5g.num_qgroups;

	value = vrb1_genmask(num_qqs_acc, num_qgs);
	for (template_idx = VRB1_SIG_DL_5G; template_idx <= VRB1_SIG_DL_5G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		vrb1_reg_fast_write(d, address, value);
	}
	/* FFT */
	num_qqs_acc += num_qgs;
	num_qgs = vrb1_conf->q_fft.num_qgroups;

	value = vrb1_genmask(num_qqs_acc, num_qgs);
	for (template_idx = VRB1_SIG_FFT; template_idx <= VRB1_SIG_FFT_LAST; template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		vrb1_reg_fast_write(d, address, value);
	}

	/* Queue Group Function mapping */
	value = 0;
	for (qg_idx = 0; qg_idx < VRB1_NUM_QGRPS_PER_WORD; qg_idx++) {
		acc = vrb1_acc_from_qgid(qg_idx, vrb1_conf);
		value |= qman_func_id[acc] << (qg_idx * 4);
	}
	vrb1_reg_fast_write(d, HWPfQmgrGrpFunction0, value);
	value = 0;
	for (qg_idx = 0; qg_idx < VRB1_NUM_QGRPS_PER_WORD; qg_idx++) {
		acc = vrb1_acc_from_qgid(qg_idx + VRB1_NUM_QGRPS_PER_WORD, vrb1_conf);
		value |= qman_func_id[acc] << (qg_idx * 4);
	}
	vrb1_reg_fast_write(d, HWPfQmgrGrpFunction1, value);

	/* Configuration of the Arbitration QGroup depth to 1 */
	for (qg_idx = 0; qg_idx < VRB1_NUM_QGRPS; qg_idx++) {
		address = HWPfQmgrArbQDepthGrp + sizeof(uint32_t) * qg_idx;
		value = 0;
		vrb1_reg_fast_write(d, address, value);
	}

	/* This pointer to ARAM (256kB) is shifted by 2 (4B per register) */
	for (qg_idx = 0; qg_idx < total_qgs; qg_idx++) {
		for (vf_idx = 0; vf_idx < vrb1_conf->num_vf_bundles; vf_idx++) {
			address = HWPfQmgrVfBaseAddr + vf_idx * sizeof(uint32_t) +
					qg_idx * sizeof(uint32_t) * 64;
			value = aram_address;
			vrb1_reg_fast_write(d, address, value);
			/* Offset ARAM Address for next memory bank
			 * - increment of 4B
			 */
			aram_address += vrb1_aq_num(qg_idx, vrb1_conf) *
					(1 << vrb1_aq_depth(qg_idx, vrb1_conf));
		}
	}

	if (aram_address > VRB1_WORDS_IN_ARAM_SIZE) {
		LOG(ERR, "ARAM Configuration not fitting %d %d",
		       aram_address, (uint32_t)VRB1_WORDS_IN_ARAM_SIZE);
		return -1;
	}

	/* Default ARAM Control configuration */
	vrb1_reg_fast_write(d, HWPfAramControlStatus, VRB1_ARAM_CONTROL);

	/* Performance tuning */
	vrb1_reg_fast_write(d, HWPfFabricI2Mdma_weight, 0x0FFF);
	vrb1_reg_fast_write(d, HWPfDma4gdlIbThld, 0x1f10);

	/* ==== HI Configuration ==== */

	/* Prevent Block on Transmit Error */
	address = HWPfHiBlockTransmitOnErrorEn;
	value = 0;
	vrb1_reg_fast_write(d, address, value);
	/* Allows to drop MSI in case FIFO is full */
	address = HWPfHiMsiDropEnableReg;
	value = 1;
	vrb1_reg_fast_write(d, address, value);
	/* Set the PF Mode register */
	address = HWPfHiPfMode;
	value = (vrb1_conf->pf_mode_en) ? VRB1_PF_VAL : 0;
	vrb1_reg_fast_write(d, address, value);

	/* QoS overflow init */
	value = 1;
	address = HWPfQosmonAEvalOverflow0;
	vrb1_reg_fast_write(d, address, value);
	address = HWPfQosmonBEvalOverflow0;
	vrb1_reg_fast_write(d, address, value);

	if (vrb1_conf->pf_mode_en)
		LOG(INFO, "Configuration in PF mode");
	else
		LOG(INFO, "Configuration in VF mode");

	/* Enable PMon */
	address = HWPfDmaPmEnable;
	value = 0x1;
	vrb1_reg_fast_write(d, address, value);

	for (vf_idx = 0; vf_idx < vrb1_conf->num_vf_bundles; vf_idx++) {
		for (acc = 0; acc < VRB1_MON_NUMS; acc++) {
			address = HWPfPermonACntrlRegVf + VRB1_PERMON_CTRL_REG_VF_OFFSET * vf_idx
					+ acc * VRB1_MON_OFFSET;
			value = 0x1; /* Reset */
			vrb1_reg_fast_write(d, address, value);
			address = HWPfPermonACntrlRegVf + VRB1_PERMON_CTRL_REG_VF_OFFSET * vf_idx
					+ acc * VRB1_MON_OFFSET;
			value = 0x2; /* Start */
			vrb1_reg_fast_write(d, address, value);
		}
	}

	for (acc = 0; acc < VRB1_MON_NUMS; acc++) {
		address = HWPfPermonACbControlFec + acc * VRB1_MON_OFFSET;
		value = 0x1;
		vrb1_reg_fast_write(d, address, value);
		value = 0x2;
		vrb1_reg_fast_write(d, address, value);
	}

	vrb1_reg_fast_write(d, HWPfPermonAControlBusMon, VRB1_BUSMON_RESET);
	vrb1_reg_fast_write(d, HWPfPermonAConfigBusMon, (0 << 8) + 1); /* Group 0-1 */
	vrb1_reg_fast_write(d, HWPfPermonASkipCountBusMon, 0);
	vrb1_reg_fast_write(d, HWPfPermonAControlBusMon, VRB1_BUSMON_START);
	vrb1_reg_fast_write(d, HWPfPermonCControlBusMon, VRB1_BUSMON_RESET);
	vrb1_reg_fast_write(d, HWPfPermonCConfigBusMon, (2 << 8) + 1); /* Group 2 */
	vrb1_reg_fast_write(d, HWPfPermonCSkipCountBusMon, 0);
	vrb1_reg_fast_write(d, HWPfPermonCControlBusMon, VRB1_BUSMON_START);

	/* Configure the FFT RAM LUT */
	vrb1_reg_fast_write(d, HWPfFftRamPageAccess, VRB1_FFT_RAM_EN + 64);
	for (i = 0; i < VRB1_FFT_RAM_SIZE; i++)
		vrb1_reg_fast_write(d, HWPfFftRamOff + i * 4, fft_lut[i]);
	vrb1_reg_fast_write(d, HWPfFftRamPageAccess, VRB1_FFT_RAM_DIS);

	/* Setup window coefficients - persistent after reset. */
	if (vrb1_conf->q_fft.num_qgroups > 0) {
		ret = vrb1_fft_reconfig(d, first_cfg, accel_dev);
		if (ret < 0)
			return ret;
	}


	/* Enabling AQueues through the Queue hierarchy*/
	for (vf_idx = 0; vf_idx < VRB1_NUM_VFS; vf_idx++) {
		for (qg_idx = 0; qg_idx < VRB1_NUM_QGRPS; qg_idx++) {
			value = 0;
			if (vf_idx < vrb1_conf->num_vf_bundles && qg_idx < total_qgs)
				value = (1 << vrb1_aq_num(qg_idx, vrb1_conf)) - 1;
			address = HWPfQmgrAqEnableVf + vf_idx * sizeof(uint32_t);
			value += (qg_idx << 16);
			vrb1_reg_fast_write(d, address, value);
		}
	}

	/* Clear explictly the VF2PF doorbells */
	for (vf_idx = 0; vf_idx < VRB1_NUM_VFS; vf_idx++) {
		status = vrb1_reg_read(d, HWPfHiVfToPfDbellVf +
				vf_idx * BB_ACC_PF_TO_VF_DBELL_REG_OFFSET);
	}

	/* Authorize the OOBMSM Telemetry. */
	vrb1_reg_fast_write(d, HWPfHiOcodeMailBox, VRB1_OCODE_REQ);

	LOG(INFO, "VRB1 configuration complete");
	return 0;
}

static int
vrb1_allocate_info_ring(void *dev)
{
	hw_device *accel_pci_dev = (hw_device *)dev;
	void *bar0addr = accel_pci_dev->bar0Addr;

	if (accel_pci_dev->info_ring)
		return 0; /* already configured */

	accel_pci_dev->info_ring = vfio_dma_alloc(dev,
			BB_ACC_INFO_RING_SIZE,
			&accel_pci_dev->info_ring_phys_addr);

	if (!accel_pci_dev->info_ring) {
		LOG(ERR, "memory allocation for info_ring failed");
		return -1;
	}
	LOG(DEBUG, "dev info ring virt:%p, phy: 0x%08lx",
			accel_pci_dev->info_ring, accel_pci_dev->info_ring_phys_addr);

	/* Do not set  for VRB1 the IR Registers in HW which conflict with HWPfHiOcodeMailBox */

	accel_pci_dev->info_ring_head =
			(vrb1_reg_read(bar0addr, HWPfHiInfoRingPointerRegPf) & 0xFFF) /
			sizeof(uint32_t);

	LOG(DEBUG, "info_ring_head: %d", accel_pci_dev->info_ring_head);

	return 0;
}

static void
vrb1_free_info_ring(void *dev)
{
	hw_device *accel_pci_dev = (hw_device *)dev;

	LOG(DEBUG, "Free info ring");

	if (!accel_pci_dev->info_ring)
		return;

	vfio_dma_free(accel_pci_dev->info_ring, accel_pci_dev->info_ring_total_size);

	accel_pci_dev->info_ring = 0;
	accel_pci_dev->info_ring_phys_addr = 0x0;
	accel_pci_dev->info_ring_head = 0x0;

	LOG(DEBUG, "Done");
}

/* There is dependency between MSI and IR registers setting
 * When MSI bit is high, the IR bits either sends both msi and the info ring or drops both.
 * When MSI bit is low, the IR bits either drops the info ring or sends it, but MSI is always sent.
 *    MSI    0   |   1    |
 *    ---------------------
 * IR 0 | MSI    | None   |
 *    1 | MSI/IR | MSI/IR |
 *    ---------------------
 */

static void
vrb1_enable_ir_regs(uint8_t *bar0addr)
{
	/* Enabling all interrupt but no IR for device error or doorbells */
	vrb1_reg_fast_write(bar0addr, HWPfHiInfoRingIntWrEnRegPf, 0x0);
	vrb1_reg_fast_write(bar0addr, HWPfHiCfgMsiIntWrEnRegPf, 0x0);
	vrb1_reg_fast_write(bar0addr, HWPfHiInfoRingVf2pfHiWrEnReg, 0x0);
	vrb1_reg_fast_write(bar0addr, HWPfHiInfoRingVf2pfLoWrEnReg, 0x0);
	vrb1_reg_fast_write(bar0addr, HWPfHiCfgMsiVf2pfHighWrEnReg, 0x0);
	vrb1_reg_fast_write(bar0addr, HWPfHiCfgMsiVf2pfLoWrEnReg, 0x0);
}

static void
vrb1_disable_ir_regs(uint8_t *bar0addr)
{
	/* disable all interrupts and IR */
	vrb1_reg_fast_write(bar0addr, HWPfHiInfoRingIntWrEnRegPf, 0x0);
	vrb1_reg_fast_write(bar0addr, HWPfHiCfgMsiIntWrEnRegPf, 0xFFFFFFFF);
	vrb1_reg_fast_write(bar0addr, HWPfHiInfoRingVf2pfLoWrEnReg, 0x0);
	vrb1_reg_fast_write(bar0addr, HWPfHiInfoRingVf2pfHiWrEnReg, 0x0);
	vrb1_reg_fast_write(bar0addr, HWPfHiCfgMsiVf2pfHighWrEnReg, 0xFFFFFFFF);
	vrb1_reg_fast_write(bar0addr, HWPfHiCfgMsiVf2pfLoWrEnReg, 0xFFFFFFFF);
}

static void
vrb1_reset_info_ring(hw_device *accel_dev)
{
	uint32_t phys_high, phys_low, i, *ring_data;
	hw_device *accel_pci_dev = (hw_device *)accel_dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;

	/* Set back the Info Ring base addresses */
	phys_high = (uint32_t) (accel_pci_dev->info_ring_phys_addr >> 32);
	phys_low = (uint32_t) (accel_pci_dev->info_ring_phys_addr);
	vrb1_reg_fast_write(bar0addr, HWPfHiInfoRingPointerRegPf, phys_high);
	vrb1_reg_fast_write(bar0addr, HWPfHiInfoRingBaseLoRegPf, phys_low);

	/* Clean up the ring and adjust pointer */
	vrb1_reg_fast_write(bar0addr, HWPfHiInfoRingPointerRegPf, 0xF000);
	accel_pci_dev->info_ring_head =
			(vrb1_reg_read(bar0addr, HWPfHiInfoRingPointerRegPf) & 0xFFF) /
			sizeof(uint32_t);
	/* Clear explicitly content of Info Ring */
	if (accel_pci_dev->info_ring != NULL) {
		for (i = 0; i < VRB1_INFO_RING_NUM_ENTRIES; i++) {
			ring_data = accel_pci_dev->info_ring + i;
			*ring_data = 0;
		}
	}
}

/* Disable queues so that PMD knows the device is not enabled */
void vrb1_disable_aqueues(void *bar0addr)
{
	unsigned int vf_idx, qg_idx, address;

	/* Disabling AQueues through the Queue hierarchy. */
	for (vf_idx = 0; vf_idx < VRB1_NUM_VFS; vf_idx++) {
		for (qg_idx = 0; qg_idx < VRB1_NUM_QGRPS; qg_idx++) {
			address = HWPfQmgrAqEnableVf + vf_idx * sizeof(uint32_t);
			vrb1_reg_fast_write(bar0addr, address, qg_idx << 16);
		}
	}
}

void vrb1_cluster_reset(void *dev)
{
	hw_device *accel_dev = (hw_device *)dev;
	void *bar0addr = accel_dev->bar0Addr;
	uint16_t template_idx, i;

	vrb1_disable_ir_regs(bar0addr);

	LOG(DEBUG, "vrb1cluster reset");
	/* Disable Qmgr Template Configuration */
	vrb1_disable_aqueues(bar0addr);
	for (template_idx = 0; template_idx < VRB1_NUM_TMPL; template_idx++) {
		vrb1_reg_fast_write(bar0addr, HWPfQmgrGrpTmplateReg4Indx +
				sizeof(uint32_t) * template_idx, 0);
	}

	/* Stop the DMA and ask Ocode to go idle and wait for 1 msec. */
	vrb1_reg_fast_write(bar0addr, HWPfHiOcodeMailBox, 0);
	vrb1_reg_fast_write(bar0addr, HWPfDmaSoftResetReg, 0x2);
	usleep(1000);
	vrb1_reg_fast_write(bar0addr, HWPfDmaSoftResetReg, 0x0);

	for (i = 0; i < 2; i++) {
		/* cluster reset */
		vrb1_reg_fast_write(bar0addr, HWPfHiCoresHardResetReg, 0xFFFFFFFF);
		vrb1_reg_fast_write(bar0addr, HWPfHi5GHardResetReg, 0xFFFFFFFF);
		vrb1_reg_fast_write(bar0addr, HWPfHiHardResetReg, 0x3FF);

		/* wait for 10 usecs */
		usleep(10);
	}

	vrb1_reset_info_ring(accel_dev);
}

int vrb1_enable_intr(void *dev)
{
	int ret;
	hw_device *accel_pci_dev = (hw_device *)dev;
	void *bar0addr = accel_pci_dev->bar0Addr;

	LOG(DEBUG, "Enable VRB1 interrupts");
	/* allocate info ring */
	ret = vrb1_allocate_info_ring(dev);
	if (ret < 0) {
		LOG(ERR, "vrb1allocate info ring failed");
		return -1;
	}

	vrb1_enable_ir_regs(bar0addr);

	LOG(DEBUG, "Done");
	return 0;
}

int
vrb1_disable_intr(void *dev)
{
	hw_device *accel_pci_dev = (hw_device *)dev;
	void *bar0addr = accel_pci_dev->bar0Addr;

	LOG(DEBUG, "Disable interrupts");

	vrb1_disable_ir_regs(bar0addr);

	vrb1_free_info_ring(dev);

	LOG(DEBUG, "Done");
	return 0;
}

int
vrb1_configure(void *dev, void *bar0addr, const char *cfg_filename, const bool first_cfg)
{
	struct vrb1_conf vrb1_conf;
	int ret;

	ret = vrb1_read_config_file(cfg_filename, &vrb1_conf);
	if (ret != 0) {
		printf("Error reading config file.\n");
		return -1;
	}

	ret = vrb1_write_config(dev, bar0addr, &vrb1_conf, first_cfg);
	if (ret != 0) {
		printf("Error writing configuration for VRB1.\n");
		return -1;
	}
	hw_device *accel_dev = (hw_device *)dev;
	accel_dev->numvfs = vrb1_conf.num_vf_bundles;
	return 0;
}

void vrb1_device_data(void *dev)
{
	uint32_t vf_idx, num, min, max;
	uint64_t avg;
	hw_device *accel_dev = (hw_device *)dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;

	LOG_RESP(INFO, "Device Status:: %d VFs\n", accel_dev->numvfs);
	for (vf_idx = 0; vf_idx < accel_dev->numvfs; vf_idx++)
		LOG_RESP(INFO, "-  VF %d %s\n", vf_idx,
			 bb_acc_device_status_str(accel_dev->dev_status[vf_idx]));
	LOG_RESP(INFO, "5GUL counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonACountVf, accel_dev->numvfs, VRB1_PMON_OFF_1);
	LOG_RESP(INFO, "5GUL counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonAKCntLoVf, accel_dev->numvfs, VRB1_PMON_OFF_1);
	LOG_RESP(INFO, "5GUL counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonACbCountFec, VRB1_5GUL_ENGS, VRB1_PMON_OFF_2);
	LOG_RESP(INFO, "5GDL counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonBCountVf, accel_dev->numvfs, VRB1_PMON_OFF_1);
	LOG_RESP(INFO, "5GDL counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonBKCntLoVf, accel_dev->numvfs, VRB1_PMON_OFF_1);
	LOG_RESP(INFO, "5GDL counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonBCbCountFec, VRB1_5GDL_ENGS, VRB1_PMON_OFF_2);
	LOG_RESP(INFO, "FFT counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonCCountVf, accel_dev->numvfs, VRB1_PMON_OFF_1);
	LOG_RESP(INFO, "FFT counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonCKCntLoVf, accel_dev->numvfs, VRB1_PMON_OFF_1);
	LOG_RESP(INFO, "FFT counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonCCbCountFec, VRB1_FFT_ENGS, VRB1_PMON_OFF_2);
	/* Bus Monitor */
	vrb1_reg_fast_write(bar0addr, HWPfPermonAControlBusMon, VRB1_BUSMON_STOP);
	min = vrb1_reg_read(bar0addr, HWPfPermonAMinLatBusMon);
	max = vrb1_reg_read(bar0addr, HWPfPermonAMaxLatBusMon);
	num = vrb1_reg_read(bar0addr, HWPfPermonATotalReqCntBusMon);
	if (num > 0) {
		avg = (vrb1_reg_read(bar0addr, HWPfPermonATotalLatLowBusMon) +
				vrb1_reg_read(bar0addr, HWPfPermonATotalLatUpperBusMon)
				* ((uint64_t)1 << 32)) / num;
		LOG_RESP(INFO, "Bus Monitor A (ns) Avg %u - Num %u - Min %d - Max %d",
				(uint32_t) avg, num, min, max);
	}
	vrb1_reg_fast_write(bar0addr, HWPfPermonAControlBusMon, VRB1_BUSMON_RESET);
	vrb1_reg_fast_write(bar0addr, HWPfPermonAControlBusMon, VRB1_BUSMON_START);
	vrb1_reg_fast_write(bar0addr, HWPfPermonCControlBusMon, VRB1_BUSMON_STOP);
	min = vrb1_reg_read(bar0addr, HWPfPermonCMinLatBusMon);
	max = vrb1_reg_read(bar0addr, HWPfPermonCMaxLatBusMon);
	num = vrb1_reg_read(bar0addr, HWPfPermonCTotalReqCntBusMon);
	if (num > 0) {
		avg = (vrb1_reg_read(bar0addr, HWPfPermonCTotalLatLowBusMon) +
				vrb1_reg_read(bar0addr, HWPfPermonCTotalLatUpperBusMon)
				* ((uint64_t)1 << 32)) / num;
		LOG_RESP(INFO, "Bus Monitor C (ns) Avg %u - Num %u - Min %d - Max %d",
				(uint32_t) avg, num, min, max);
	}
	vrb1_reg_fast_write(bar0addr, HWPfPermonCControlBusMon, VRB1_BUSMON_RESET);
	vrb1_reg_fast_write(bar0addr, HWPfPermonCControlBusMon, VRB1_BUSMON_START);

}

