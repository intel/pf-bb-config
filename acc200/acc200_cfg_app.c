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
#include "acc200_cfg_app.h"
#include "acc200_fft_lut.h"
#include "acc200_pf_enum.h"
#include "bb_acc.h"
#include "bb_acc_log.h"

static acc200_ir_int_type_info histatus[32] = {
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

static acc200_ir_int_type_info extError[32] = {
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
acc200_reg_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
	usleep(1000);
}

static void
acc200_reg_fast_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
}

static uint32_t
acc200_reg_read(uint8_t *mmio_base, uint32_t offset)
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
acc200_pfvf(void *dev, unsigned int vf_index, unsigned int payload)
{
	hw_device *accel_dev = (hw_device *)dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;

	LOG(DEBUG, "Doorbell vf2pf");

	if (payload == REQ_DEV_STATUS) {
		acc200_reg_fast_write(bar0addr, HWPfHiPfToVfDbellVf +
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
	if (payload == REQ_DEV_NEW && accel_dev->dev_status[vf_index] == RTE_BBDEV_DEV_CONFIGURED) {
		accel_dev->dev_status[vf_index] = RTE_BBDEV_DEV_ACTIVE;
		LOG(INFO, "VF Device %u becomes active", vf_index);
	}

	LOG(DEBUG, "Done");
}

static bool
acc200_device_error(void *dev, int int_src)
{
	uint32_t err_status;
	uint16_t i;
	hw_device *accel_dev = (hw_device *)dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;
	bool fatal_error = false;

	err_status = acc200_reg_read(bar0addr, HWPfHiErrStatusReg);
	for (i = 0; i < 32; i++) {
		if (((err_status >> i) & 0x1) == 0x1) {
			if (histatus[i].fatal)
				fatal_error = true;
			LOG(ERR, "HI Device Error: %d %s", i, histatus[i].name);
		}
	}

	err_status = acc200_reg_read(bar0addr, HWPfDmaAxiStatus);
	if ((err_status & 0x1) == 0x1) {
		fatal_error = true;
		LOG(ERR, "Stop AXI asserted");
	}
	err_status = acc200_reg_read(bar0addr, HWPfHiIosf2axiErrLogReg);
	for (i = 0; i < 32; i++)
		if (((err_status >> i) & 0x1) == 0x1)
			LOG(ERR, "Ext Device Error: %d %s", i, extError[i].name);

	/* Clear error injection */
	if (fatal_error) {
		acc200_reg_fast_write(bar0addr, HWPfHiErrInjectReg, 0);
		acc200_reg_fast_write(bar0addr, HWPfAramControlStatus, ACC200_ARAM_CONTROL);
		acc200_reg_fast_write(bar0addr, HWPfDmaCmplTmOutCnt, 0xFFFFFFFF);
		acc200_reg_fast_write(bar0addr, HWPfDmaProcTmOutCnt, ACC200_PROC_TIMEOUT);
		acc200_reg_fast_write(bar0addr, HWPfQmgrAramWatchdogCount, 0xFFFFFFFF);
		acc200_reg_fast_write(bar0addr, HWPfQmgrAxiWatchdogCount, 0xFFFFFFFF);
		acc200_reg_fast_write(bar0addr, HWPfQmgrProcessWatchdogCount, 0xFFFFFFFF);
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
			acc200_reg_fast_write(bar0addr, HWPfHiIosf2axiErrLogReg, 0xFFFFFFFF);
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
acc200_irq_handler(void *dev)
{
	hw_device *accel_pci_dev = (hw_device *)dev;
	uint8_t *bar0addr = accel_pci_dev->bar0Addr;
	uint32_t vf_index, status;
	/* No infoRing data available */

	/* Check first VF2PF comms */
	for (vf_index = 0; vf_index < ACC200_NUM_VFS; vf_index++) {
		status = acc200_reg_read(bar0addr, HWPfHiVfToPfDbellVf +
				BB_ACC_PF_TO_VF_DBELL_REG_OFFSET * vf_index);
		if (status != 0) {
			acc200_pfvf(dev, vf_index, status);
			LOG(DEBUG, "door bell %d %d", vf_index, status);
		}
	}
	/* Check for device error */
	acc200_device_error(dev, ACC200_ITR_UNDEFINED);
	return 0;
}

/**
 * Retrieve ACC200 device configuration from file. The file's content must keep
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
 * @param acc200_conf
 *   Pointer to structure that will hold ACC200 configuration
 *
 * @return
 *   Zero on success, negative value on failure.
statir void
acc200_reg_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
	usleep(1000);
}

static void
acc200_reg_fast_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
}

static uint32_t
acc200_reg_read(uint8_t *mmio_base, uint32_t offset)
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
acc200_read_config_file(const char *arg_cfg_filename,
		struct acc200_conf *acc200_conf)
{
	bool unsafe_path = cfg_file_check_path_safety(arg_cfg_filename);
	if (unsafe_path == true) {
		printf("error, config file path \"%s\" is not safe",
				arg_cfg_filename);
		return -1;
	} else
		return acc200_parse_conf_file(arg_cfg_filename, acc200_conf);
}

enum {UL_4G = 0, UL_5G, DL_4G, DL_5G, FFT, NUM_ACC};

/* Return the que topology for a Queue Group Index */
void
acc200_qtopFromAcc(struct q_topology_t **qtop, int acc_enum,
		struct acc200_conf *acc200_conf)
{
	struct q_topology_t *p_qtop;
	p_qtop = NULL;
	switch (acc_enum) {
	case UL_4G:
		p_qtop = &(acc200_conf->q_ul_4g);
		break;
	case UL_5G:
		p_qtop = &(acc200_conf->q_ul_5g);
		break;
	case DL_4G:
		p_qtop = &(acc200_conf->q_dl_4g);
		break;
	case DL_5G:
		p_qtop = &(acc200_conf->q_dl_5g);
		break;
	case FFT:
		p_qtop = &(acc200_conf->q_fft);
		break;
	default:
		/* NOTREACHED */
		printf("Unexpected error evaluating acc200_qtopFromAcc %d",
				acc_enum);
		break;
	}
	*qtop = p_qtop;
}

static int bb_acc200_fft_coeff_setup(void *d, struct acc200_conf *acc200_conf)
{
	int16_t page_idx = 0;
	int16_t offset = ACC200_FFT_FIRST_OFFSET;
	int16_t *g_tdwin_coff = (int16_t *)fft_coeff_binfile;
	int i = 0;

	if (acc200_conf->q_fft.num_qgroups > 0) {
		acc200_reg_write(d, HWPfFftRamPageAccess, ACC200_FFT_RAM_EN);
		for (i = 0; i < ACC200_LUT_SIZE; i++) {
			acc200_reg_fast_write(d, HWPfFftRamOff + offset * sizeof(uint32_t),
						 g_tdwin_coff[i]);
			offset++;
			if ((offset % ACC200_FFT_PAGE_SIZE) == 0) {
				offset = 0;
				page_idx++;
				acc200_reg_fast_write(d, HWPfFftRamPageAccess,
							 ACC200_FFT_RAM_EN +
						page_idx);
			}
		}
		acc200_reg_write(d, HWPfFftRamPageAccess, ACC200_FFT_RAM_DIS);
	}
	return 0;
}



/* Return the accelerator enum for a Queue Group Index */
int
acc200_acc_from_qgid(int qg_idx, struct acc200_conf *acc200_conf)
{
	int acc_qg[ACC200_NUM_QGRPS];
	int num_qgroups_per_fn[NUM_ACC];
	int acc, qgid, qg_index = 0;

	for (qgid = 0; qgid < ACC200_NUM_QGRPS; qgid++)
		acc_qg[qgid] = 0;
	num_qgroups_per_fn[UL_4G] = acc200_conf->q_ul_4g.num_qgroups;
	num_qgroups_per_fn[UL_5G] = acc200_conf->q_ul_5g.num_qgroups;
	num_qgroups_per_fn[DL_4G] = acc200_conf->q_dl_4g.num_qgroups;
	num_qgroups_per_fn[DL_5G] = acc200_conf->q_dl_5g.num_qgroups;
	num_qgroups_per_fn[FFT] = acc200_conf->q_fft.num_qgroups;
	for (acc = UL_4G;  acc < NUM_ACC; acc++)
		for (qgid = 0; qgid < num_qgroups_per_fn[acc]; qgid++)
			acc_qg[qg_index++] = acc;
	acc = acc_qg[qg_idx];
	return acc;
}

/* Return the que topology for a Queue Group Index */
void
acc200_qtop_from_acc(struct q_topology_t **qtop, int acc_enum,
		     struct acc200_conf *acc200_conf)
{
	struct q_topology_t *p_qtop;

	p_qtop = NULL;
	switch (acc_enum) {
	case UL_4G:
		p_qtop = &acc200_conf->q_ul_4g;
		break;
	case UL_5G:
		p_qtop = &acc200_conf->q_ul_5g;
		break;
	case DL_4G:
		p_qtop = &acc200_conf->q_dl_4g;
		break;
	case DL_5G:
		p_qtop = &acc200_conf->q_dl_5g;
		break;
	case FFT:
		p_qtop = &acc200_conf->q_fft;
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
acc200_aq_depth(int qg_idx, struct acc200_conf *acc200_conf)
{
	struct q_topology_t *q_top = NULL;
	int acc_enum = acc200_acc_from_qgid(qg_idx, acc200_conf);

	acc200_qtop_from_acc(&q_top, acc_enum, acc200_conf);
	if (!q_top)
		return 0;
	return q_top->aq_depth_log2;
}

/* Return the AQ depth for a Queue Group Index */
int
acc200_aq_num(int qg_idx, struct acc200_conf *acc200_conf)
{
	struct q_topology_t *q_top = NULL;
	int acc_enum = acc200_acc_from_qgid(qg_idx, acc200_conf);

	acc200_qtop_from_acc(&q_top, acc_enum, acc200_conf);
	if (!q_top)
		return 0;
	return q_top->num_aqs_per_groups;
}

/* Generate Qmgr templates mask */
uint32_t acc200_genmask(int num_qqs_acc, int num_qgs)
{
	if (num_qgs < 1)
		return 0;
	else
		return GENMASK(num_qgs + num_qqs_acc - 1, num_qqs_acc);
}

static int
acc200_write_config(void *dev, void *mapaddr, struct acc200_conf *acc200_conf)
{
	uint32_t value, address, status;
	int qg_idx, template_idx, vf_idx, acc, i;
	/* QMGR_ARAM - memory internal to ACC200 */
	uint32_t aram_address = 0;
	int rlim = 0;
	int alen = 1;
	int timestamp_en = 0;
	int num_qgs;
	int num_qqs_acc = 0;
	int num_engines = 0;
	int total_qgs = 0;
	int qman_func_id[8] = {0, 2, 1, 3, 4, 0, 0, 0};
	uint8_t *d = mapaddr;

	/* Check we are already out of PG */
	status = acc200_reg_read(d, HWPfHiSectionPowerGatingAck);
	if (status > 0) {
		if (status != ACC200_PG_MASK_0) {
			LOG(ERR, "Unexpected status %x %x", status, ACC200_PG_MASK_0);
			return -ENODEV;
		}
		/* Clock gate sections that will be un-PG */
		acc200_reg_write(d, HWPfHiClkGateHystReg, ACC200_CLK_DIS);
		/* Un-PG required sections */
		acc200_reg_write(d, HWPfHiSectionPowerGatingReq, ACC200_PG_MASK_1);
		status = acc200_reg_read(d, HWPfHiSectionPowerGatingAck);
		if (status != ACC200_PG_MASK_1) {
			LOG(ERR, "Unexpected status %x %x", status, ACC200_PG_MASK_1);
			return -ENODEV;
		}
		acc200_reg_write(d, HWPfHiSectionPowerGatingReq, ACC200_PG_MASK_2);
		status = acc200_reg_read(d, HWPfHiSectionPowerGatingAck);
		if (status != ACC200_PG_MASK_2) {
			LOG(ERR, "Unexpected status %x %x", status, ACC200_PG_MASK_2);
			return -ENODEV;
		}
		acc200_reg_write(d, HWPfHiSectionPowerGatingReq, ACC200_PG_MASK_3);
		status = acc200_reg_read(d, HWPfHiSectionPowerGatingAck);
		if (status != ACC200_PG_MASK_3) {
			LOG(ERR, "Unexpected status %x %x", status, ACC200_PG_MASK_3);
			return -ENODEV;
		}
		/* Enable clocks for all sections */
		acc200_reg_write(d, HWPfHiClkGateHystReg, ACC200_CLK_EN);
	}

	/* Explicitly releasing AXI as this may be stopped after PF FLR/BME */
	address = HWPfDmaAxiControl;
	value = 1;
	acc200_reg_write(d, address, value);

	/* Set the fabric mode */
	address = HWPfFabricM2iBufferReg;
	value = ACC200_FABRIC_MODE;
	acc200_reg_write(d, address, value);

	/* Set default descriptor signature */
	address = HWPfDmaDescriptorSignatuture;
	value = 0;
	acc200_reg_write(d, address, value);

	/* Enable the Error Detection in DMA */
	value = ACC200_CFG_DMA_ERROR;
	address = HWPfDmaErrorDetectionEn;
	acc200_reg_write(d, address, value);

	/* AXI Cache configuration */
	value = ACC200_CFG_AXI_CACHE;
	address = HWPfDmaAxcacheReg;
	acc200_reg_write(d, address, value);

	/* AXI Response configuration */
	acc200_reg_write(d, HWPfDmaCfgRrespBresp, 0x0);

	/* Default DMA Configuration (Qmgr Enabled) */
	address = HWPfDmaConfig0Reg;
	value = 0;
	acc200_reg_write(d, address, value);
	address = HWPfDmaQmanen;
	value = 0;
	acc200_reg_write(d, address, value);

	/* Default RLIM/ALEN configuration */
	rlim = 0;
	alen = 1;
	timestamp_en = 0;
	address = HWPfDmaConfig1Reg;
	value = (1 << 31) + (rlim << 8) + (timestamp_en << 6) + alen;
	acc200_reg_write(d, address, value);

	/* Default FFT configuration */
	address = HWPfFftConfig0;
	value = ACC200_FFT_CFG_0;
	acc200_reg_write(d, address, value);

	/* Configure the FFT ECC Mask */
	acc200_reg_write(d, HwPfFftParityMaskReg0, 1 << 31);
	acc200_reg_write(d, HwPfFftParityMaskReg1, 1 << 31);

	/* Configure DMA Qmanager addresses */
	address = HWPfDmaQmgrAddrReg;
	value = HWPfQmgrEgressQueuesTemplate;
	acc200_reg_write(d, address, value);

	/* Enable time out counters */
	acc200_reg_write(d, HWPfDmaCmplTmOutCnt, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfDmaProcTmOutCnt, ACC200_PROC_TIMEOUT);
	acc200_reg_write(d, HWPfDmaConfigPtoutOutEn, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfDmaStatusToutProcess, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfDmaConfigCtoutOutDataEn, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfDmaConfigCtoutOutDescrEn, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfDmaConfigUnexpComplDataEn, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfDmaConfigUnexpComplDescrEn, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfDmaConfigPtoutOutEn, 0xFFFFFFFF);

	acc200_reg_write(d, HWPfQmgrAramWatchdogCount, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfQmgrAramWatchdogCounterEn, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfQmgrAxiWatchdogCount, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfQmgrAxiWatchdogCounterEn, 0xFFFFFFFF);
	for (address = 0; address < 8; address++)
		acc200_reg_write(d, HWPfQmgrProcessWatchdogCounterFunc +
				sizeof(uint32_t) * address, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfQmgrProcessWatchdogCount, 0xFFFFFFFF);
	acc200_reg_write(d, HWPfQmgrProcessWatchdogCounterEn, 0xFF);

	/* ===== Qmgr Configuration ===== */
	/* Configuration of the AQueue Depth QMGR_GRP_0_DEPTH_LOG2 for UL */
	total_qgs = acc200_conf->q_ul_4g.num_qgroups +
		acc200_conf->q_ul_5g.num_qgroups +
		acc200_conf->q_dl_4g.num_qgroups +
		acc200_conf->q_dl_5g.num_qgroups +
		acc200_conf->q_fft.num_qgroups;

	LOG(INFO, "Queue Groups UL4G %d DL4G %d UL5G %d DL5G %d FFT %d",
			acc200_conf->q_ul_4g.num_qgroups, acc200_conf->q_dl_4g.num_qgroups,
			acc200_conf->q_ul_5g.num_qgroups, acc200_conf->q_dl_5g.num_qgroups,
			acc200_conf->q_fft.num_qgroups);

	for (qg_idx = 0; qg_idx < ACC200_NUM_QGRPS; qg_idx++) {
		address = HWPfQmgrDepthLog2Grp + sizeof(uint32_t) * qg_idx;
		value = acc200_aq_depth(qg_idx, acc200_conf);
		acc200_reg_write(d, address, value);
		address = HWPfQmgrTholdGrp + sizeof(uint32_t) * qg_idx;
		value = (1 << 16) + (1 << (acc200_aq_depth(qg_idx, acc200_conf) - 1));
		acc200_reg_write(d, address, value);
	}

	/* Template Priority in incremental order */
	for (template_idx = 0; template_idx < ACC200_NUM_TMPL; template_idx++) {
		address = HWPfQmgrGrpTmplateReg0Indx + sizeof(uint32_t) * template_idx;
		value = ACC200_TMPL_PRI_0;
		acc200_reg_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg1Indx + sizeof(uint32_t) * template_idx;
		value = ACC200_TMPL_PRI_1;
		acc200_reg_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg2Indx + sizeof(uint32_t) * template_idx;
		value = ACC200_TMPL_PRI_2;
		acc200_reg_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg3Indx + sizeof(uint32_t) * template_idx;
		value = ACC200_TMPL_PRI_3;
		acc200_reg_write(d, address, value);
	}

	address = HWPfQmgrGrpPriority;
	value = ACC200_CFG_QMGR_HI_P;
	acc200_reg_write(d, address, value);

	/* Template Configuration */
	for (template_idx = 0; template_idx < ACC200_NUM_TMPL; template_idx++) {
		value = 0;
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		acc200_reg_write(d, address, value);
	}
	/* 4GUL */
	num_qgs = acc200_conf->q_ul_4g.num_qgroups;
	num_qqs_acc = 0;
	value = acc200_genmask(num_qqs_acc, num_qgs);
	for (template_idx = ACC200_SIG_UL_4G; template_idx <= ACC200_SIG_UL_4G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		acc200_reg_write(d, address, value);
	}
	/* 5GUL */
	num_qqs_acc += num_qgs;
	num_qgs = acc200_conf->q_ul_5g.num_qgroups;
	num_engines = 0;
	value = acc200_genmask(num_qqs_acc, num_qgs);
	for (template_idx = ACC200_SIG_UL_5G; template_idx <= ACC200_SIG_UL_5G_LAST;
			template_idx++) {
		/* Check engine power-on status */
		address = HWPfFecUl5gIbDebugReg + ACC200_ENGINE_OFFSET * template_idx;
		status = (acc200_reg_read(d, address) >> 4) & 0x7;
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		if (status == 1) {
			acc200_reg_write(d, address, value);
			num_engines++;
		} else {
			acc200_reg_write(d, address, 0);
		}
	}
	LOG(DEBUG, "Number of 5GUL engines %d", num_engines);
	/* 4GDL */
	num_qqs_acc += num_qgs;
	num_qgs = acc200_conf->q_dl_4g.num_qgroups;

	value = acc200_genmask(num_qqs_acc, num_qgs);
	for (template_idx = ACC200_SIG_DL_4G; template_idx <= ACC200_SIG_DL_4G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		acc200_reg_write(d, address, value);
	}
	/* 5GDL */
	num_qqs_acc += num_qgs;
	num_qgs = acc200_conf->q_dl_5g.num_qgroups;

	value = acc200_genmask(num_qqs_acc, num_qgs);
	for (template_idx = ACC200_SIG_DL_5G; template_idx <= ACC200_SIG_DL_5G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		acc200_reg_write(d, address, value);
	}
	/* FFT */
	num_qqs_acc += num_qgs;
	num_qgs = acc200_conf->q_fft.num_qgroups;

	value = acc200_genmask(num_qqs_acc, num_qgs);
	for (template_idx = ACC200_SIG_FFT; template_idx <= ACC200_SIG_FFT_LAST; template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx + sizeof(uint32_t) * template_idx;
		acc200_reg_write(d, address, value);
	}

	/* Queue Group Function mapping */
	value = 0;
	for (qg_idx = 0; qg_idx < ACC200_NUM_QGRPS_PER_WORD; qg_idx++) {
		acc = acc200_acc_from_qgid(qg_idx, acc200_conf);
		value |= qman_func_id[acc] << (qg_idx * 4);
	}
	acc200_reg_write(d, HWPfQmgrGrpFunction0, value);
	value = 0;
	for (qg_idx = 0; qg_idx < ACC200_NUM_QGRPS_PER_WORD; qg_idx++) {
		acc = acc200_acc_from_qgid(qg_idx + ACC200_NUM_QGRPS_PER_WORD, acc200_conf);
		value |= qman_func_id[acc] << (qg_idx * 4);
	}
	acc200_reg_write(d, HWPfQmgrGrpFunction1, value);

	/* Configuration of the Arbitration QGroup depth to 1 */
	for (qg_idx = 0; qg_idx < ACC200_NUM_QGRPS; qg_idx++) {
		address = HWPfQmgrArbQDepthGrp + sizeof(uint32_t) * qg_idx;
		value = 0;
		acc200_reg_write(d, address, value);
	}

	/* This pointer to ARAM (256kB) is shifted by 2 (4B per register) */
	for (qg_idx = 0; qg_idx < total_qgs; qg_idx++) {
		for (vf_idx = 0; vf_idx < acc200_conf->num_vf_bundles; vf_idx++) {
			address = HWPfQmgrVfBaseAddr + vf_idx * sizeof(uint32_t) +
					qg_idx * sizeof(uint32_t) * 64;
			value = aram_address;
			acc200_reg_write(d, address, value);
			/* Offset ARAM Address for next memory bank
			 * - increment of 4B
			 */
			aram_address += acc200_aq_num(qg_idx, acc200_conf) *
					(1 << acc200_aq_depth(qg_idx, acc200_conf));
		}
	}

	if (aram_address > ACC200_WORDS_IN_ARAM_SIZE) {
		LOG(ERR, "ARAM Configuration not fitting %d %d",
		       aram_address, (uint32_t)ACC200_WORDS_IN_ARAM_SIZE);
		return -1;
	}

	/* Default ARAM Control configuration */
	acc200_reg_write(d, HWPfAramControlStatus, ACC200_ARAM_CONTROL);

	/* Performance tuning */
	acc200_reg_write(d, HWPfFabricI2Mdma_weight, 0x0FFF);
	acc200_reg_write(d, HWPfDma4gdlIbThld, 0x1f10);

	/* ==== HI Configuration ==== */

	/* Prevent Block on Transmit Error */
	address = HWPfHiBlockTransmitOnErrorEn;
	value = 0;
	acc200_reg_write(d, address, value);
	/* Allows to drop MSI in case FIFO is full */
	address = HWPfHiMsiDropEnableReg;
	value = 1;
	acc200_reg_write(d, address, value);
	/* Set the PF Mode register */
	address = HWPfHiPfMode;
	value = (acc200_conf->pf_mode_en) ? ACC200_PF_VAL : 0;
	acc200_reg_write(d, address, value);

	/* QoS overflow init */
	value = 1;
	address = HWPfQosmonAEvalOverflow0;
	acc200_reg_write(d, address, value);
	address = HWPfQosmonBEvalOverflow0;
	acc200_reg_write(d, address, value);

	if (acc200_conf->pf_mode_en)
		LOG(INFO, "Configuration in PF mode");
	else
		LOG(INFO, "Configuration in VF mode");

	/* Enable PMon */
	address = HWPfDmaPmEnable;
	value = 0x1;
	acc200_reg_write(d, address, value);

	for (vf_idx = 0; vf_idx < acc200_conf->num_vf_bundles; vf_idx++) {
		address = HWPfPermonACntrlRegVf + ACC200_PERMON_CTRL_REG_VF_OFFSET * vf_idx;
		value = 0x1; /* Reset */
		acc200_reg_write(d, address, value);
		address = HWPfPermonACntrlRegVf + ACC200_PERMON_CTRL_REG_VF_OFFSET * vf_idx;
		value = 0x2; /* Start */
		acc200_reg_write(d, address, value);
		address = HWPfPermonBCntrlRegVf + ACC200_PERMON_CTRL_REG_VF_OFFSET * vf_idx;
		value = 0x1; /* Reset */
		acc200_reg_write(d, address, value);
		address = HWPfPermonBCntrlRegVf + ACC200_PERMON_CTRL_REG_VF_OFFSET * vf_idx;
		value = 0x2; /* Start */
		acc200_reg_write(d, address, value);
	}

	address = HWPfPermonACbControlFec;
	value = 0x1;
	acc200_reg_write(d, address, value);
	value = 0x2;
	acc200_reg_write(d, address, value);
	address = HWPfPermonBCbControlFec;
	value = 0x1;
	acc200_reg_write(d, address, value);
	value = 0x2;
	acc200_reg_write(d, address, value);

	acc200_reg_write(d, HWPfPermonAControlBusMon, 1);
	acc200_reg_write(d, HWPfPermonAConfigBusMon, 1);
	acc200_reg_write(d, HWPfPermonASkipCountBusMon, 0);
	acc200_reg_write(d, HWPfPermonAControlBusMon, 2);

#ifdef QOS_ENABLING
	int register_offset, gbr, xl;

	/* ==== QoS Configuration ==== */
	/* Loop for both clusters */
	register_offset = HWPfQosmonBCntrlReg - HWPfQosmonACntrlReg;

	for (xl = 0; xl < 2; xl++) {
		/* Enable+Clear QosMon and Block Size granularity to 2^3 */
		address = HWPfQosmonACntrlReg + xl * register_offset;
		value = (1 << 8) + 1; /* Enable + Reset */
		acc200_reg_write(d, address, value);
	}
	usleep(1000);

	address = HWPfDmaQosEnable;
	value = 1;
	acc200_reg_write(d, address, value);

	for (xl = 0; xl < 2; xl++) {
		/* Enable+Clear QosMon and Block Size granularity to 2^3 */
		address = HWPfQosmonACntrlReg + xl * register_offset;
		value = (1 << 9) + (1 << 4) + (3 << 1);
		acc200_reg_write(d, address, value);
	}

	for (xl = 0; xl < 2; xl++) {
		/* Clock Divider */
		int clock_divider = 5;

		address = HWPfQosmonADivTerm + xl * register_offset;
		value = (clock_divider - 1);
		acc200_reg_write(d, address, value);
		/* Terminal Counter resolution - 1us */
		address = HWPfQosmonATickTerm + xl * register_offset;
		value = 400 / clock_divider - 1;
		acc200_reg_write(d, address, value);
		/* Evaluation Window - 5us */
		address = HWPfQosmonAEvalTerm + xl * register_offset;
		value = 5 - 1;
		acc200_reg_write(d, address, value);
		/* Average Window - 500ms */
		address = HWPfQosmonAAveTerm + xl * register_offset;
		value = 100 - 1;
		acc200_reg_write(d, address, value);
	}

	for (vf_idx = 0; vf_idx < acc200_conf->num_vf_bundles; vf_idx++) {
		/* UL */
		gbr = acc200_conf->arb_ul_5g[0].gbr_threshold1;

		if (gbr == 0)
			gbr = 1;
		address = HWPfQosmonARemThres1Vf + VF_OFFSET_QOS * vf_idx;
		value = gbr;

		acc200_reg_write(d, address, value);
		address = HWPfQosmonAThres2Vf + VF_OFFSET_QOS * vf_idx;
		value = 0;
		acc200_reg_write(d, address, value);
		address = HWPfQosmonAWeiFracVf + VF_OFFSET_QOS * vf_idx;
		value = ((acc200_conf->arb_ul_5g[0].round_robin_weight + 1) << 24) / gbr;
		acc200_reg_write(d, address, value);

		/* DL */
		gbr = acc200_conf->arb_dl_5g[0].gbr_threshold1;
		if (gbr == 0)
			gbr = 1;
		address = HWPfQosmonBRemThres1Vf + VF_OFFSET_QOS * vf_idx;
		value = gbr;
		acc200_reg_write(d, address, value);
		address = HWPfQosmonBThres2Vf + VF_OFFSET_QOS * vf_idx;
		value = gbr / 2;
		acc200_reg_write(d, address, value);
		address = HWPfQosmonBWeiFracVf + VF_OFFSET_QOS * vf_idx;
		value = ((acc200_conf->arb_dl_5g[0].round_robin_weight + 1) << 24) / gbr;
		acc200_reg_write(d, address, value);
	}
	/* Explicitly reset remaining bundles parameters */
	for (vf_idx = acc200_conf->num_vf_bundles; vf_idx < ACC200_NUM_VFS; vf_idx++) {
		value = 0;
		address = HWPfQosmonARemThres1Vf + VF_OFFSET_QOS * vf_idx;
		acc200_reg_write(d, address, value);
		address = HWPfQosmonAThres2Vf + VF_OFFSET_QOS * vf_idx;
		acc200_reg_write(d, address, value);
		address = HWPfQosmonAWeiFracVf + VF_OFFSET_QOS * vf_idx;
		acc200_reg_write(d, address, value);
		address = HWPfQosmonBRemThres1Vf + VF_OFFSET_QOS * vf_idx;
		acc200_reg_write(d, address, value);
		address = HWPfQosmonBThres2Vf + VF_OFFSET_QOS * vf_idx;
		acc200_reg_write(d, address, value);
		address = HWPfQosmonBWeiFracVf + VF_OFFSET_QOS * vf_idx;
		acc200_reg_write(d, address, value);
	}

	/* QoS LDPC Configuration */
	for (xl = 0; xl < 4; xl++) {
		/* Disable feature for now*/
		int qos_iter_b = 20;
		int qos_iter_a = 10;
		int qos_frac_step = 10;
		int qos_frac_shift = 10;
		int qos_threshold = 10;

		address = HWPfQosmonAIterationConfig0Low + 8 * xl;
		value = qos_frac_shift + (qos_frac_step << 4);
		acc200_reg_write(d, address, value);
		address = HWPfQosmonAIterationConfig0High + 8 * xl;
		value = qos_iter_b + (qos_iter_a << 6) + (qos_threshold << 12);
		acc200_reg_write(d, address, value);
	}

	/* Enable Qos Scaling */
	address = HWPfDmaQosScale;
	value = 0x3;
	acc200_reg_write(d, address, value);

	for (xl = 0; xl < 2; xl++) {
		/* Enable QosMon with block Size granularity to 2^3 */
		address = HWPfQosmonACntrlReg + xl * register_offset;
		value = (1 << 8) + (1 << 4) + (3 << 1);
		acc200_reg_write(d, address, value);
	}
#endif /* QOS_ENABLING */

	/* Configure the FFT RAM LUT */
	acc200_reg_write(d, HWPfFftRamPageAccess, ACC200_FFT_RAM_EN + 64);
	for (i = 0; i < ACC200_FFT_RAM_SIZE; i++)
		acc200_reg_write(d, HWPfFftRamOff + i * 4, fft_lut[i]);
	acc200_reg_write(d, HWPfFftRamPageAccess, ACC200_FFT_RAM_DIS);

	/* setup window coefficients */
	bb_acc200_fft_coeff_setup(d, acc200_conf);

	/* Enabling AQueues through the Queue hierarchy*/
	for (vf_idx = 0; vf_idx < ACC200_NUM_VFS; vf_idx++) {
		for (qg_idx = 0; qg_idx < ACC200_NUM_QGRPS; qg_idx++) {
			value = 0;
			if (vf_idx < acc200_conf->num_vf_bundles && qg_idx < total_qgs)
				value = (1 << acc200_aq_num(qg_idx, acc200_conf)) - 1;
			address = HWPfQmgrAqEnableVf + vf_idx * sizeof(uint32_t);
			value += (qg_idx << 16);
			acc200_reg_write(d, address, value);
		}
	}

	/* Clear explictly the VF2PF doorbells */
	for (vf_idx = 0; vf_idx < ACC200_NUM_VFS; vf_idx++) {
		status = acc200_reg_read(d, HWPfHiVfToPfDbellVf +
				BB_ACC_PF_TO_VF_DBELL_REG_OFFSET * vf_idx);
	}

	/* Authorize the OOBMSM Telemetry. */
	acc200_reg_write(d, HWPfHiOcodeMailBox, ACC200_OCODE_REQ);

	LOG(INFO, "ACC200 configuration complete");
	return 0;
}

static int
acc200_allocate_info_ring(void *dev)
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

	/* Do not set  for ACC200 the IR Registers in HW which conflict with HWPfHiOcodeMailBox */

	accel_pci_dev->info_ring_head =
			(acc200_reg_read(bar0addr, HWPfHiInfoRingPointerRegPf) & 0xFFF) /
			sizeof(uint32_t);

	LOG(DEBUG, "info_ring_head: %d", accel_pci_dev->info_ring_head);

	return 0;
}

static void
acc200_free_info_ring(void *dev)
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
acc200_enable_ir_regs(uint8_t *bar0addr)
{
	/* Enabling all interrupt but no IR for device error or doorbells */
	acc200_reg_write(bar0addr, HWPfHiInfoRingIntWrEnRegPf, 0x0);
	acc200_reg_write(bar0addr, HWPfHiCfgMsiIntWrEnRegPf, 0x0);
	acc200_reg_write(bar0addr, HWPfHiInfoRingVf2pfHiWrEnReg, 0x0);
	acc200_reg_write(bar0addr, HWPfHiInfoRingVf2pfLoWrEnReg, 0x0);
	acc200_reg_write(bar0addr, HWPfHiCfgMsiVf2pfHighWrEnReg, 0x0);
	acc200_reg_write(bar0addr, HWPfHiCfgMsiVf2pfLoWrEnReg, 0x0);
}

static void
acc200_disable_ir_regs(uint8_t *bar0addr)
{
	/* disable all interrupts and IR */
	acc200_reg_write(bar0addr, HWPfHiInfoRingIntWrEnRegPf, 0x0);
	acc200_reg_write(bar0addr, HWPfHiCfgMsiIntWrEnRegPf, 0xFFFFFFFF);
	acc200_reg_write(bar0addr, HWPfHiInfoRingVf2pfLoWrEnReg, 0x0);
	acc200_reg_write(bar0addr, HWPfHiInfoRingVf2pfHiWrEnReg, 0x0);
	acc200_reg_write(bar0addr, HWPfHiCfgMsiVf2pfHighWrEnReg, 0xFFFFFFFF);
	acc200_reg_write(bar0addr, HWPfHiCfgMsiVf2pfLoWrEnReg, 0xFFFFFFFF);
}

static void
acc200_reset_info_ring(hw_device *accel_dev)
{
	uint32_t phys_high, phys_low, i, *ring_data;
	hw_device *accel_pci_dev = (hw_device *)accel_dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;

	/* Set back the Info Ring base addresses */
	phys_high = (uint32_t) (accel_pci_dev->info_ring_phys_addr >> 32);
	phys_low = (uint32_t) (accel_pci_dev->info_ring_phys_addr);
	acc200_reg_write(bar0addr, HWPfHiInfoRingPointerRegPf, phys_high);
	acc200_reg_write(bar0addr, HWPfHiInfoRingBaseLoRegPf, phys_low);

	/* Clean up the ring and adjust pointer */
	acc200_reg_write(bar0addr, HWPfHiInfoRingPointerRegPf, 0xF000);
	accel_pci_dev->info_ring_head =
			(acc200_reg_read(bar0addr, HWPfHiInfoRingPointerRegPf) & 0xFFF) /
			sizeof(uint32_t);
	/* Clear explicitly content of Info Ring */
	if (accel_pci_dev->info_ring != NULL) {
		for (i = 0; i < ACC200_INFO_RING_NUM_ENTRIES; i++) {
			ring_data = accel_pci_dev->info_ring + i;
			*ring_data = 0;
		}
	}
}

/* Disable queues so that PMD knows the device is not enabled */
void acc200_disable_aqueues(void *bar0addr)
{
	unsigned int vf_idx, qg_idx, address;

	/* Disabling AQueues through the Queue hierarchy. */
	for (vf_idx = 0; vf_idx < ACC200_NUM_VFS; vf_idx++) {
		for (qg_idx = 0; qg_idx < ACC200_NUM_QGRPS; qg_idx++) {
			address = HWPfQmgrAqEnableVf + vf_idx * sizeof(uint32_t);
			acc200_reg_write(bar0addr, address, qg_idx << 16);
		}
	}
}

void acc200_cluster_reset(void *dev)
{
	hw_device *accel_dev = (hw_device *)dev;
	void *bar0addr = accel_dev->bar0Addr;
	uint16_t template_idx, i;

	acc200_disable_ir_regs(bar0addr);

	LOG(DEBUG, "acc200 cluster reset");
	/* Disable Qmgr Template Configuration */
	acc200_disable_aqueues(bar0addr);
	for (template_idx = 0; template_idx < ACC200_NUM_TMPL; template_idx++) {
		acc200_reg_write(bar0addr, HWPfQmgrGrpTmplateReg4Indx +
				sizeof(uint32_t) * template_idx, 0);
	}

	/* Stop the DMA and ask Ocode to go idle and wait for 1 msec. */
	acc200_reg_write(bar0addr, HWPfHiOcodeMailBox, 0);
	acc200_reg_write(bar0addr, HWPfDmaSoftResetReg, 0x2);
	usleep(1000);
	acc200_reg_write(bar0addr, HWPfDmaSoftResetReg, 0x0);

	for (i = 0; i < 2; i++) {
		/* cluster reset */
		acc200_reg_write(bar0addr, HWPfHiCoresHardResetReg, 0xFFFFFFFF);
		acc200_reg_write(bar0addr, HWPfHi5GHardResetReg, 0xFFFFFFFF);
		acc200_reg_write(bar0addr, HWPfHiHardResetReg, 0x3FF);

		/* wait for 10 usecs */
		usleep(10);
	}

	acc200_reset_info_ring(accel_dev);
}

int acc200_enable_intr(void *dev)
{
	int ret;
	hw_device *accel_pci_dev = (hw_device *)dev;
	void *bar0addr = accel_pci_dev->bar0Addr;

	LOG(DEBUG, "Enable ACC200 interrupts");
	/* allocate info ring */
	ret = acc200_allocate_info_ring(dev);
	if (ret < 0) {
		LOG(ERR, "acc200 allocate info ring failed");
		return -1;
	}

	acc200_enable_ir_regs(bar0addr);

	LOG(DEBUG, "Done");
	return 0;
}

int
acc200_disable_intr(void *dev)
{
	hw_device *accel_pci_dev = (hw_device *)dev;
	void *bar0addr = accel_pci_dev->bar0Addr;

	LOG(DEBUG, "Disable interrupts");

	acc200_disable_ir_regs(bar0addr);

	acc200_free_info_ring(dev);

	LOG(DEBUG, "Done");
	return 0;
}

int
acc200_configure(void *dev, void *bar0addr, const char *cfg_filename)
{
	struct acc200_conf acc200_conf;
	int ret;

	ret = acc200_read_config_file(cfg_filename, &acc200_conf);
	if (ret != 0) {
		printf("Error reading config file.\n");
		return -1;
	}

	ret = acc200_write_config(dev, bar0addr, &acc200_conf);
	if (ret != 0) {
		printf("Error writing configuration for ACC200.\n");
		return -1;
	}
	hw_device *accel_dev = (hw_device *)dev;
	accel_dev->numvfs = acc200_conf.num_vf_bundles;
	return 0;
}

void acc200_device_data(void *dev)
{
	uint32_t vf_idx;
	uint32_t num;
	uint64_t avg;

	hw_device *accel_dev = (hw_device *)dev;

	uint8_t *bar0addr = accel_dev->bar0Addr;

	LOG(INFO, "Device Status:: %d VFs\n", accel_dev->numvfs);
	for (vf_idx = 0; vf_idx < accel_dev->numvfs; vf_idx++)
		LOG(INFO, "-  VF %d %s\n", vf_idx,
			 bb_acc_device_status_str(accel_dev->dev_status[vf_idx]));
	LOG(INFO, "5GUL counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonACountVf, accel_dev->numvfs, ACC200_PMON_OFF_1);
	LOG(INFO, "5GUL counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonAKCntLoVf, accel_dev->numvfs, ACC200_PMON_OFF_1);
	LOG(INFO, "5GUL counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonACbCountFec, ACC200_5GUL_ENGS, ACC200_PMON_OFF_2);
	LOG(INFO, "5GDL counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonBCountVf, accel_dev->numvfs, ACC200_PMON_OFF_1);
	LOG(INFO, "5GDL counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonBKCntLoVf, accel_dev->numvfs, ACC200_PMON_OFF_1);
	LOG(INFO, "5GDL counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonBCbCountFec, ACC200_5GDL_ENGS, ACC200_PMON_OFF_2);
	LOG(INFO, "FFT counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonCCountVf, accel_dev->numvfs, ACC200_PMON_OFF_1);
	LOG(INFO, "FFT counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonCKCntLoVf, accel_dev->numvfs, ACC200_PMON_OFF_1);
	LOG(INFO, "FFT counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonCCbCountFec, ACC200_FFT_ENGS, ACC200_PMON_OFF_2);
	/* Bus Monitor */
	acc200_reg_write(bar0addr, HWPfPermonAControlBusMon, 0);
	num = acc200_reg_read(bar0addr, HWPfPermonATotalReqCntBusMon);
	if (num > 0) {
		avg = (acc200_reg_read(bar0addr, HWPfPermonATotalLatLowBusMon) +
				acc200_reg_read(bar0addr, HWPfPermonATotalLatUpperBusMon)
				* ((uint64_t)1 << 32)) / num;
		LOG(INFO, "Bus Monitor Avg %u - Num %u",
				(uint32_t) avg, num);
	}
	acc200_reg_write(bar0addr, HWPfPermonAControlBusMon, 1);
	acc200_reg_write(bar0addr, HWPfPermonAControlBusMon, 2);
}

