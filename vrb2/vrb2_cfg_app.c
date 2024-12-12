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
#include "vrb2_cfg_app.h"
#include "vrb2_pf_enum.h"
#include "bb_acc.h"
#include "bb_acc_log.h"

/* Keep track of device stepping used. */
bool b0_variant;

/* Info Ring Interrupt Source */
static vrb2_ir_int_type_info irdata[32] = {
	{"QMGR_AQ_OVERFLOW",          true,  false}, /*/ 0 */
	{"DOORBELL_VFTOPF",           true,  false}, /* 1 */
	{"ILLEGAL_FORMAT",            true,  false}, /* 2 */
	{"QMGR_DISABLED_ACCESS",      true,  false}, /* 3 */
	{"QMGR_AQ_OVER_THRESHOLD",    true,  false}, /* 4 */
	{"4G_DMA_DL_DESC_IRQ",        true,  false}, /* 5 */
	{"4G_DMA_UL_DESC_IRQ",        true,  false}, /* 6 */
	{"FFT_DESC_IRQ",              true,  false}, /* 7 */
	{"5G_DMA_UL_DESC_IRQ",        true,  false}, /* 8 */
	{"5G_DMA_DL_DESC_IRQ",        true,  false}, /* 9 */
	{"MLD_DESC_IRQ",              true,  false}, /* 10 */
	{"PAGE_FAULT_ATS_ERR",        true,  false}, /* 11 */
	{"ARAM_ECC_1BIT_ERROR",       false, false}, /* 12 */
	{"PARITY_ERROR",              true, true},  /* 13 */
	{"QMGR_INTREQ_FIFO_OVERFLOW", false, false}, /* 14 */
	{"QMGR_ERROR_DETECTED",       true,  true},  /* 15 */
	{"UNUSED", false}, /* 16 */
	{"UNUSED", false}, /* 17 */
	{"UNUSED", false}, /* 18 */
	{"UNUSED", false}, /* 19 */
	{"UNUSED", false}, /* 20 */
	{"UNUSED", false}, /* 21 */
	{"ARAM_ACCESS_ERROR",          true,  false}, /* 22 */
	{"QMGR_ARAM_ALMOST_FULL",      false, false}, /* 23 */
	{"5G_EXTRA_COMPLETION_RECVD",  false, true},  /* 24 */
	{"5G_COMPLETION_READ_TIMEOUT", false, true},  /* 25 */
	{"CORE_HANG_EVENT",            true,  false}, /* 26 */
	{"DMA_CLUSTER_HANG_DETECTED",  false, true},  /* 27 */
	{"DOWN_STREAM_HANG_DETECTED",  true,  true},  /* 28 */
	{"UNUSED", false},
	{"UNUSED", false},
	{"UNUSED", false},
};

/* HI Error Register */
static vrb2_ir_int_type_info histatus[32] = {
	{"DMA_EXTRA_READ_STATUS",    true, true},     /* 0 */
	{"DMA_READ_TIMEOUT",         true, true},     /* 1 */
	{"DMA_HW_ERROR_STATUS",      true, false},    /* 2 */
	{"FEC_SLICE_ERROR_STATUS",   true, false},    /* 3 */
	{"PROCESS_TIMEOUT",          true, false},    /* 4 */
	{"DOWNSTREAM_HANG_DETECT",   true, true},     /* 5 */
	{"UNUSED", false},
	{"UNUSED", false},
	{"UNUSED", false},
	{"UNUSED", false},
	{"UNUSED", false},
	{"DMA_SM_TO_ERROR",          true, true},    /* 11 */
	{"PAGE_FAULT_HDRERR_STATUS", true, false},   /* 12 */
	{"PAGE_FAULT_ATSERR_STATUS", true, false},   /* 13 */
	{"CHR_COMPLETED",            true, false},   /* 14 */
	{"CHR_FAILED_STATUS",        true, true},    /* 15 */
	{"QMGR_ACC_TIMEOUT",         true, true},    /* 16 */
	{"ARAM_PROCESS_TIMEOUT",     true, true},    /* 17 */
	{"QMGR_PROCESS_TIMEOUT",     true, true},    /* 18 */
	{"MSI_FIFO_OVERFLOW",        true, false},   /* 19 */
	{"QMGR_ARAM_ALMOST_FULL",    true, false},   /* 20 */
	{"QMGR_AQ_DROPPED_STATUS",   true, false},   /* 21 */
	{"ARAM_ACCESS_ERR_STATUS",   true, true},    /* 22 */
	{"ARAM_ECC_ERR_STATUS",      true, false},   /* 23 */
	{"PARITY_FATAL_STATUS",      true, true},    /* 24 */
	{"PARITY_NONFATAL_STATUS",   true, false},   /* 25 */
	{"DMA_CLUSTER_HANG",         true, true},    /* 26 */
	{"INTERRUPT_FIFO_OVERFLOW",  true, false},   /* 27 */
	{"AXI_PARITY_ERR_STATUS",    true, true},    /* 28 */
	{"HI_1BIT_ECC_ERR_STATUS",   true, false},   /* 29 */
	{"HI_MBIT_ECC_ERR_STATUS",   true, true},    /* 30 */
	{"HI_MEM_PARITY_STATUS",     true, true},    /* 31 */
};

/* HI_IOSF2AXI Extra Error Register */
static vrb2_ir_int_type_info extError[32] = {
	{"DMADL_PAR_ERR_STATUS", true },       /* 0  */
	{"DMAUL_PAR_ERR_STATUS", true },       /* 1  */
	{"QMGR_PAR_ERR_STATUS", true },        /* 2  */
	{"ARAM_PAR_ERR_STATUS", true },        /* 3  */
	{"PAR_ERR_STATUS_5G", true },          /* 4  */
	{"HI_MEM_PARITY_STATUS", true },       /* 5  */
	{"HI_AXI_PARITY_STATUS", true },       /* 6  */
	{"QMGR_ARAM_ALMOST_FULL", false },     /* 7  */
	{"QMGR_AQ_OVERFLOW_STATUS", false },   /* 8  */
	{"UNUSED", false },                    /* 9  */
	{"PAGE_FAULT_STATUS", false},          /* 10 */
	{"DMA_CLUSTER_HANG_STATUS", false},    /* 11 */
	{"DS_HANG_STATUS", false},             /* 12 */
	{"UNUSED", false},                     /* 13 */
	{"UNUSED", false},                     /* 14 */
	{"UNUSED", false},                     /* 15 */
	{"UNUSED", false},                     /* 16 */
	{"UNUSED", false},                     /* 17 */
	{"CHR_DMA_EVENT", false},              /* 18 */
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

static void
vrb2_reg_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
	usleep(1);
}

static void
vrb2_reg_fast_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
}

static uint32_t
vrb2_reg_read(uint8_t *mmio_base, uint32_t offset)
{

	void *reg_addr = mmio_base + offset;
	uint32_t ret = *((volatile uint32_t *)(reg_addr));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	ret = __bswap_32(ret);
#endif
	return ret;
}

/* Write doorbell register for a given VF for PF2VF communications. */
static void
vrb2_db_write(void *dev, unsigned int vf_index, uint32_t msg)
{
	hw_device *accel_dev = (hw_device *)dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;

	vrb2_reg_fast_write(bar0addr, HWPfHiPfToVfDbellVf
				+ vf_index * BB_ACC_PF_TO_VF_DBELL_REG_OFFSET,
				msg);
	/* Dummy write to prevent back to back DB. */
	vrb2_reg_fast_write(bar0addr, HWPfHiModuleVersionReg, 0);
}

/* Manage PF to VF communications */
static void
vrb2_pfvf(void *dev, unsigned int vf_index, unsigned int payload)
{
	hw_device *accel_dev = (hw_device *)dev;
	unsigned int window_index;

	LOG(DEBUG, "Doorbell vf2pf %d", payload);

	if (payload == REQ_DEV_STATUS) {
		vrb2_db_write(dev, vf_index, accel_dev->dev_status[vf_index]);
		if (accel_dev->prev_status[vf_index] != accel_dev->dev_status[vf_index]) {
			LOG(INFO, "Report Device Status VF: %u Status %lu %s",
				vf_index, accel_dev->dev_status[vf_index],
				bb_acc_device_status_str(accel_dev->dev_status[vf_index]));
			accel_dev->prev_status[vf_index] = accel_dev->dev_status[vf_index];
		}
		/* Clear back when read once */
		if (accel_dev->dev_status[vf_index] == RTE_BBDEV_DEV_RESTART_REQ ||
				accel_dev->dev_status[vf_index] == RTE_BBDEV_DEV_RECONFIG_REQ ||
				accel_dev->dev_status[vf_index] == RTE_BBDEV_DEV_CORRECT_ERR)
			accel_dev->dev_status[vf_index] = RTE_BBDEV_DEV_CONFIGURED;
	}
	/* Reports a version number based on the used FFT LUT binary file. */
	if (payload == REQ_DEV_LUT_VER) {
		vrb2_db_write(dev, vf_index, (uint32_t)accel_dev->fft_version_md5sum);
	}
	/* Reports Window sizes based on the used FFT LUT binary file. */
	if ((payload & REQ_DEV_MASK) == REQ_DEV_FFT_WIN_SIZE) {
		window_index = payload >> 16;
		vrb2_db_write(dev, vf_index, (uint32_t)accel_dev->fft_win_size[window_index]);
	}
	if ((payload & REQ_DEV_MASK) == REQ_DEV_FFT_WIN_START) {
		window_index = payload >> 16;
		vrb2_db_write(dev, vf_index, (uint32_t)accel_dev->fft_win_start[window_index]);
	}
	if (payload == REQ_DEV_NEW && accel_dev->dev_status[vf_index] == RTE_BBDEV_DEV_CONFIGURED) {
		accel_dev->dev_status[vf_index] = RTE_BBDEV_DEV_ACTIVE;
		LOG(INFO, "VF Device %u becomes active", vf_index);
	}

	LOG(DEBUG, "Done");
}

/* Report in log error captured in Error status registers */
static bool
vrb2_check_error_reg(uint8_t *bar0addr)
{
	uint32_t err_status, i;
	bool fatal_error = false;

	err_status = vrb2_reg_read(bar0addr, HWPfHiErrStatusReg);
	for (i = 0; i < 32; i++) {
		if (((err_status >> i) & 0x1) == 0x1) {
			if (histatus[i].fatal)
				fatal_error = true;
			LOG(ERR, "HI Device Error: %d %s", i, histatus[i].name);
		}
	}
	err_status = vrb2_reg_read(bar0addr, HWPfHiIosf2axiErrLogReg);
	for (i = 0; i < 32; i++)
		if (((err_status >> i) & 0x1) == 0x1)
			LOG(ERR, "Ext Device Error: %d %s", i, extError[i].name);
	/* Removing the CHR flag */
	vrb2_reg_write(bar0addr, HWPfHiIosf2axiErrLogReg, 1 << VRB2_DMA_ERROR_OR_CHR);
	err_status = vrb2_reg_read(bar0addr, HWPfDmaAxiStatus);
	if ((err_status & 0x1) == 0x1) {
		fatal_error = true;
		LOG(ERR, "Stop AXI asserted");
	}
	err_status = vrb2_reg_read(bar0addr, HWPfDmaClusterHangStatus);
	if ((err_status & 0x1) == 0x1) {
		fatal_error = true;
		LOG(ERR, "DMA Cluster Timer expired");
	}

	return fatal_error;
}

/* Configure or reconfigure a given FFT engine. */
static int
vrb2_fft_reconfig(uint8_t *d, int template_idx, bool lut_programming, hw_device *accel_pci_dev)
{
	int16_t gTDWinCoff[VRB2_LUT_SIZE], ret, offset, i, pageIdx;
	const char *lut_filename;
	FILE *fp;
	bool unsafe_path;

	/* Default FFT configuration. */
	vrb2_reg_write(d, HWPfFftConfig0 + template_idx * 0x1000,
			b0_variant ? VRB2_FFT_CFG_0 : VRB2_FFT_CFG_0_A0);
	vrb2_reg_write(d, HWPfFftParityMask8 + template_idx * 0x1000,
			b0_variant ? VRB2_FFT_ECC : VRB2_FFT_ECC_A0);

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
		LOG(ERR, "  Error reading from %s on engine %d ", lut_filename, template_idx);
		return -ENOENT;
	}
	ret = fread((char *)gTDWinCoff, VRB2_LUT_SIZE * 2, 1, fp);
	if (ret <= 0) {
		LOG(ERR, "  Error reading from %s on engine %d ", lut_filename, template_idx);
		fclose(fp);
		return -ENOENT;
	}
	if (template_idx == 0)
		LOG(INFO, "  FFT Window coeffs preloading from %s on engine %d ",
				lut_filename, template_idx);

	offset = VRB2_FFT_FIST_OFFSET;
	pageIdx = 0;
	vrb2_reg_fast_write(d, HWPfFftRamPageAccess + template_idx * 0x1000, VRB2_FFT_RAM_EN);
	for (i = 0; i < VRB2_LUT_SIZE; i++) {
		vrb2_reg_fast_write(d, HWPfFftRamOff + offset *
				VRB2_BYTES_IN_WORD + template_idx * 0x1000,
				gTDWinCoff[i]);
		offset++;
		if ((offset % VRB2_FFT_PAGE_SIZE) == 0) {
			offset = 0;
			pageIdx++;
			fflush(stdout);
			vrb2_reg_fast_write(d, HWPfFftRamPageAccess + template_idx * 0x1000,
					VRB2_FFT_RAM_EN + pageIdx);
		}
	}

	vrb2_reg_fast_write(d, HWPfFftRamPageAccess + template_idx * 0x1000, VRB2_FFT_RAM_DIS);
	fclose(fp);

	if (template_idx != 0)
		return 0;

	vrb_fft_win_check(gTDWinCoff, accel_pci_dev);

	vrb_fft_lut_md5sum(lut_filename, accel_pci_dev);

	return 0;
}

/* When MSI for core hang recovery is triggered or completed
 * Check for relevant status registers and when applicable enable back the recovered eng
 */
static void
vrb2_core_hang_recovery(void *dev, uint16_t error_type)
{
	hw_device *accel_dev = (hw_device *)dev;
	uint8_t template_index, *bar0addr = accel_dev->bar0Addr;
	uint16_t engine_idx;
	uint32_t status;
	int eng;

	/* Disabling of Error injection done in *vrb2_device_error* */

	status = vrb2_reg_read(bar0addr, HWPfDmaQmanen);
	if (status > 0)
		LOG(ERR, "HWPfDmaQmanen bitmap updated by HW: %X", status);

	for (eng = 0; eng < 32; eng++) {
		status = vrb2_reg_read(bar0addr, HWPfDmaChrStatus + eng * 0x10);
		if (status > 0)
			LOG(ERR, "HWPfDmaChrStatus: engine %d State Machine 0x%x", eng, status);
	}
	status = vrb2_reg_read(bar0addr, HWPfDmaStatusMemParErr);
	if (status > 0)
		LOG(ERR, "HWPfDmaStatusMemParErr: %x", status);
	status = vrb2_reg_read(bar0addr, HWPfDmaStatusToutProcess);
	if (status > 0)
		LOG(ERR, "HWPfDmaStatusToutProcess bitmap: %x", status);
	bb_acc_set_all_device_status(accel_dev, RTE_BBDEV_DEV_CORRECT_ERR);
	if (error_type == VRB2_ERR_TYPE_PROC_TO) {
		LOG(ERR, "Wait for engine recovery to be completed");
	} else {
		engine_idx = error_type - VRB2_ERR_TYPE_OFFSET;
		if ((engine_idx >= VRB2_IDMAP_FFT) && (engine_idx <= VRB2_IDMAP_FFT_LAST)) {
			LOG(ERR, "Reconfigure FFT engine %d.", engine_idx - VRB2_IDMAP_FFT);
			vrb2_fft_reconfig(bar0addr, engine_idx - VRB2_IDMAP_FFT, false, accel_dev);
		}
		if (engine_idx <= VRB2_IDMAP_MLD_LAST) {
			uint8_t mapping[VRB2_IDMAP_MLD_LAST + 1] = {
					4, 6, 7, 9, 11, 13, 24, 26, 27,
					8, 10, 12, 14, 15, 25, 29, 31, 0, 1, 2, 3, 5};
			template_index = mapping[engine_idx];
			/* Clear the time out report for that engine. */
			vrb2_reg_write(bar0addr, HWPfDmaStatusToutProcess, 1 << engine_idx);
			/* Enable back Qmgr for that engine. */
			vrb2_reg_write(bar0addr, HWPfDmaQmanenSelect, 1 << template_index);
			vrb2_reg_write(bar0addr, HWPfDmaQmanen, 0x0);
			LOG(ERR, "Recovery completed for engine %d on template %d",
					engine_idx, template_index);
		}
	}
}

/* Disable int/ir for a given interrupt source. */
static void vrb2_disable_int_src(uint8_t *bar0addr, int int_src)
{
	uint32_t mask;

	mask = vrb2_reg_read(bar0addr, HWPfHiInfoRingIntWrEnRegPf);
	mask &= (0xFFFFFFFF - (1ULL << int_src));
	LOG(INFO, "Disable the Interrupt source %d new mask %x", int_src, mask);
	vrb2_reg_write(bar0addr, HWPfHiInfoRingIntWrEnRegPf, mask);
}

/* Manage device error detection and recovery.
   Returns true when the rest of the IR can be skipped due to reconfiguration.
*/
static bool
vrb2_device_error(void *dev, int int_src)
{
	uint16_t eng_type;
	hw_device *accel_dev = (hw_device *)dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;
	bool fatal_error = false, exit_ir = false;

	/* Flag fatal error based on interrupt source */
	if (irdata[int_src].fatal)
		fatal_error = true;
	/* Flag fatal error based on error register */
	if (vrb2_check_error_reg(bar0addr))
		fatal_error = true;

	/* Clear error injection */
	vrb2_reg_write(bar0addr, HWPfHiErrInjectReg, 0);
	vrb2_reg_write(bar0addr, HWPfAramControlStatus, VRB2_ARAM_CONTROL);
	vrb2_reg_write(bar0addr, HWPfDmaCmplTmOutCnt, 0xFFFFFFFF);
	for (eng_type = 0; eng_type < VRB2_ENG_TYPE_NUM; eng_type++)
		vrb2_reg_write(bar0addr, HWPfDmaProcTmOutCnt + eng_type * 4, VRB2_PROC_TIMEOUT);
	vrb2_reg_write(bar0addr, HWPfQmgrAramWatchdogCount, VRB2_QMGR_ARAM_TIMEOUT);
	vrb2_reg_write(bar0addr, HWPfQmgrAxiWatchdogCount, VRB2_QMGR_AXI_TIMEOUT);
	vrb2_reg_write(bar0addr, HWPfQmgrProcessWatchdogCount, VRB2_QMGR_TIMEOUT);
	vrb2_reg_write(bar0addr, HWPfDmaClusterHangThld, VRB2_CLUST_TIMEOUT);
	if (b0_variant)
		vrb2_reg_write(bar0addr, HWPfDmaClusterCtrl, 0x1);

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
			vrb2_reg_fast_write(bar0addr, HWPfHiIosf2axiErrLogReg, 0xFFFFFFFF);
			/* reset and reconfigure the device */
			if (accel_dev->device_reset_using_flr == DEVICE_RESET_USING_FLR) {
				LOG(INFO, "FLR and reconfig");
				if (bb_acc_dev_reset_and_reconfig(accel_dev))
					LOG(ERR, "FLR and reconfig failed");
			} else {
				LOG(INFO, "Cluster reset and reconfig");
				if (bb_acc_cluster_reset_and_reconfig(accel_dev))
					LOG(ERR, "Cluster reset and reconfig failed");
			}
			/* Exit IR since the device was reconfigured. */
			exit_ir = true;
		}
	}

	if (fatal_error && !exit_ir) {
		/* In case of fatal error but not recovered, need to hide this very error
		source so that to prevent flooding the IR with the same cause. */
		vrb2_disable_int_src(bar0addr, int_src);
		if (int_src == VRB2_DMA_CLUSTER_HANG_DETECTED)
			vrb2_disable_int_src(bar0addr, VRB2_CORE_HANG_DETECTED);
	}

	if (!fatal_error)
		LOG(DEBUG, "Non fatal error");

	return exit_ir;
}

int
vrb2_irq_handler(void *dev)
{
	hw_device *accel_pci_dev = (hw_device *)dev;
	uint32_t *info_ring = (uint32_t *) (accel_pci_dev->info_ring);
	uint32_t *ring_data = info_ring +
				(accel_pci_dev->info_ring_head & VRB2_INFO_RING_MASK);
	uint8_t *bar0addr = accel_pci_dev->bar0Addr;
	uint16_t curr_head, error_type, engine_idx;
	uint32_t int_nb, detailed_info, status;

	LOG(DEBUG, "info_ring_head: %d", accel_pci_dev->info_ring_head);
	LOG(DEBUG, "dev info ring virt:%p", info_ring);
	LOG(DEBUG, "dev ring data virt:%p", ring_data);

	if (!FIELD_GET(INFO_RING_VALID, *ring_data)) {
		/* read the info_ring_head from HW */
		curr_head = (vrb2_reg_read(bar0addr, HWPfHiInfoRingPointerRegPf)
				& VRB2_INFO_RING_PTR_MASK) / sizeof(uint32_t);
		if (curr_head != accel_pci_dev->info_ring_head) {
			accel_pci_dev->info_ring_head = curr_head;
			ring_data = info_ring + (accel_pci_dev->info_ring_head &
				VRB2_INFO_RING_MASK);
		}
		LOG(DEBUG, "No Information Ring available %d %d\n",
			curr_head, accel_pci_dev->info_ring_head);

		if (vrb2_device_error(dev, VRB2_ITR_UNDEFINED))
			return 0;

	}
	while (FIELD_GET(INFO_RING_VALID, *ring_data)) {
		int_nb = FIELD_GET(INFO_RING_INT_NB, *ring_data);
		if (int_nb >= 32) {
			LOG(INFO, "MSI: unexpected Event: %d  msi_0=%ld loop=%ld ",
				int_nb,
				FIELD_GET(INFO_RING_MSI_0, *ring_data),
				FIELD_GET(INFO_RING_LOOP, *ring_data));
			if (vrb2_device_error(dev, VRB2_ITR_UNDEFINED))
				return 0;
		} else if (irdata[int_nb].use_det_info) {
			switch (int_nb) {
			case VRB2_DOORBELL_VFTOPF:
				vrb2_pfvf(dev, FIELD_GET(INFO_RING_VF2PF, *ring_data),
						FIELD_GET(INFO_RING_DET_INFO, *ring_data));
				break;

			case VRB2_QMGR_AQ_OVERFLOW:
			case VRB2_QMGR_DISABLED_ACCESS:
			case VRB2_QMGR_AQ_OVER_THRESHOLD:
			case VRB2_4G_DMA_DL_DESC_IRQ:
			case VRB2_4G_DMA_UL_DESC_IRQ:
			case VRB2_FFT_DESC_IRQ:
			case VRB2_5G_DMA_DL_DESC_IRQ:
			case VRB2_5G_DMA_UL_DESC_IRQ:
			case VRB2_MLD_DESCRIPTOR_IRQ:
				/* These should not happen*/
				LOG(INFO,
					"Unexpected MSI: Event: %s aq_id = 0x%lx qg_id=0x%lx vf_id=0x%lx",
					irdata[int_nb].name,
					FIELD_GET(INFO_RING_DET_INFO_AQ_ID, *ring_data),
					FIELD_GET(INFO_RING_DET_INFO_QG_ID, *ring_data),
					FIELD_GET(INFO_RING_DET_INFO_VF_ID, *ring_data));
				if (vrb2_device_error(dev, int_nb))
					return 0;
				break;

			case VRB2_PAGE_FAULT_ATS_ERROR:
				LOG(INFO, "MSI: Event: %s err_type=%ld aq_id = 0x%lx "
					"qg_id=0x%lx vf_id=0x%lx",
					irdata[int_nb].name,
					FIELD_GET(INFO_RING_ERR_TYPE, *ring_data),
					FIELD_GET(INFO_RING_DET_INFO_AQ_ID, *ring_data),
					FIELD_GET(INFO_RING_DET_INFO_QG_ID, *ring_data),
					FIELD_GET(INFO_RING_DET_INFO_VF_ID, *ring_data));
				if (vrb2_device_error(dev, int_nb))
					return 0;
				break;

			case VRB2_PARITY_ERR:
				LOG(INFO, "MSI: Event: %s err_type=%ld Info = 0x%lx",
					irdata[int_nb].name,
					FIELD_GET(INFO_RING_ERR_TYPE, *ring_data),
					FIELD_GET(INFO_RING_DET_INFO, *ring_data));
				if (vrb2_device_error(dev, int_nb))
					return 0;
				break;

			case VRB2_QMGR_ERROR_DETECTED:
				error_type = FIELD_GET(INFO_RING_ERR_TYPE, *ring_data);
				if (error_type == 4)
					LOG(INFO,
						"MSI: Event: %s err_type =0x%lx AXI Time Out Info= 0x%lx",
						irdata[int_nb].name,
						FIELD_GET(INFO_RING_ERR_TYPE, *ring_data),
						FIELD_GET(INFO_RING_DET_INFO, *ring_data));
				else if (error_type == 5)
					LOG(INFO,
						"MSI: Event: %s err_type =0x%lx ARAM Time Out Info= 0x%lx",
						irdata[int_nb].name,
						FIELD_GET(INFO_RING_ERR_TYPE, *ring_data),
						FIELD_GET(INFO_RING_DET_INFO, *ring_data));
				else
					LOG(INFO,
						"MSI: Event: %s err_type =0x%lx Process Time Out Info= 0x%lx",
						irdata[int_nb].name,
						FIELD_GET(INFO_RING_ERR_TYPE, *ring_data),
						FIELD_GET(INFO_RING_DET_INFO, *ring_data));
				if (vrb2_device_error(dev, int_nb))
					return 0;
				break;

			case VRB2_5G_EXTRA_COMPLETION_RECVD:
			case VRB2_5G_COMPLETION_READ_TIMEOUT:
				LOG(INFO, "MSI: Event: %s fec slice num = 0x%lx",
					irdata[int_nb].name,
					FIELD_GET(INFO_RING_DET_INFO_FEC_SLICE, *ring_data));
				if (vrb2_device_error(dev, int_nb))
					return 0;
				break;

			case VRB2_CORE_HANG_DETECTED:
				error_type = FIELD_GET(INFO_RING_ERR_TYPE, *ring_data);
				detailed_info = FIELD_GET(INFO_RING_DET_INFO, *ring_data);
				LOG(INFO, "MSI: Event: %s Raw %x err_type=0x%x",
					irdata[int_nb].name, *ring_data,
					error_type);
				/* Such error types are actually DMA Fatal errors. */
				if (error_type <= VRB2_CORE_DMA_ERROR)
					int_nb = VRB2_DMA_CLUSTER_HANG_DETECTED;
				/* Core hang recovery failure handled as fatal error. */
				if (detailed_info == VRB2_CORE_RECOVERY_FAILURE) {
					int_nb = VRB2_DMA_CLUSTER_HANG_DETECTED;
					LOG(ERR, "Core hang recovery failure on engine %d",
							error_type - VRB2_ERR_TYPE_OFFSET);
				}

				/* Special handling to catch data corruption after CHR */
				if (error_type >= VRB2_ERR_TYPE_OFFSET) {
					engine_idx = error_type - VRB2_ERR_TYPE_OFFSET;
					status = vrb2_reg_read(bar0addr, HWPfDmaStatusToutProcess);
					/* Core hang reported without processing time out . */
					if ((status & (1 << engine_idx)) == 0) {
						LOG(ERR, "CHR report is due to parity fatal error");
						int_nb = VRB2_DMA_CLUSTER_HANG_DETECTED;
					}
				}

				if (vrb2_device_error(dev, int_nb))
					return 0;
				vrb2_core_hang_recovery(dev, error_type);
				break;

			case VRB2_DOWN_STREAM_HANG_DETECTED:
				LOG(INFO, "MSI: Event: %s slave_id=%ld master_id=%ld "
					"write=%ld trans_id=%ld",
					irdata[int_nb].name,
					FIELD_GET(INFO_RING_DET_INFO_SLAVE_ID, *ring_data),
					FIELD_GET(INFO_RING_DET_INFO_MASTER_ID, *ring_data),
					FIELD_GET(INFO_RING_DET_INFO_WRITE, *ring_data),
					FIELD_GET(INFO_RING_DET_INFO_TRANS_ID, *ring_data));
				if (vrb2_device_error(dev, int_nb))
					return 0;
				break;

			default:
				/* Other cases */
				LOG(INFO, "MSI: Event: int_nb=%d %s detailed_info = 0x%lx",
					int_nb, irdata[int_nb].name,
					FIELD_GET(INFO_RING_DET_INFO, *ring_data));
				if (vrb2_device_error(dev, int_nb))
					return 0;
				break;
			}
		} else {
			/* No additional information */
			LOG(INFO, "MSI: Event: %d  %s msi_0=%ld loop=%ld vf2pf = %ld Ptr %d",
				int_nb, irdata[int_nb].name,
				FIELD_GET(INFO_RING_MSI_0, (uint32_t) *ring_data),
				FIELD_GET(INFO_RING_LOOP, (uint32_t) *ring_data),
				FIELD_GET(INFO_RING_VF2PF, (uint32_t) *ring_data),
				accel_pci_dev->info_ring_head);
			if (vrb2_device_error(dev, int_nb))
				return 0;
		}

		/* Initialize Info Ring entry and move forward */
		*ring_data = 0;
		++accel_pci_dev->info_ring_head;
		ring_data = info_ring + (accel_pci_dev->info_ring_head &
			 VRB2_INFO_RING_MASK);
	}

	return 0;
}

/**
 * Retrieve VRB2 device configuration from file. The file's content must keep
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
 * @param vrb2_conf
 *   Pointer to structure that will hold VRB2 configuration
 *
 * @return
 *   Zero on success, negative value on failure.
 */
static int
vrb2_read_config_file(const char *arg_cfg_filename,
		struct vrb2_conf *vrb2_conf)
{
	bool unsafe_path = cfg_file_check_path_safety(arg_cfg_filename);
	if (unsafe_path == true) {
		LOG(ERR, "config file path \"%s\" is not safe", arg_cfg_filename);
		return -1;
	} else
		return vrb2_parse_conf_file(arg_cfg_filename, vrb2_conf);
}

enum {UL_4G = 0, UL_5G, DL_4G, DL_5G, FFT, MLD, NUM_ACC};

/* Return the accelerator enum for a Queue Group Index */
int
vrb2_accFromQgid(int qg_idx, struct vrb2_conf *vrb2_conf)
{
	int accQg[VRB2_NUM_QGRPS];
	int NumQGroupsPerFn[NUM_ACC];
	int acc, qgIdx, qgIndex = 0;
	for (qgIdx = 0; qgIdx < VRB2_NUM_QGRPS; qgIdx++)
		accQg[qgIdx] = 0;
	NumQGroupsPerFn[UL_4G] = vrb2_conf->q_ul_4g.num_qgroups;
	NumQGroupsPerFn[UL_5G] = vrb2_conf->q_ul_5g.num_qgroups;
	NumQGroupsPerFn[DL_4G] = vrb2_conf->q_dl_4g.num_qgroups;
	NumQGroupsPerFn[DL_5G] = vrb2_conf->q_dl_5g.num_qgroups;
	NumQGroupsPerFn[FFT] = vrb2_conf->q_fft.num_qgroups;
	NumQGroupsPerFn[MLD] = vrb2_conf->q_mld.num_qgroups;
	for (acc = UL_4G;  acc < NUM_ACC; acc++)
		for (qgIdx = 0; qgIdx < NumQGroupsPerFn[acc]; qgIdx++)
			accQg[qgIndex++] = acc;
	acc = accQg[qg_idx];
	return acc;
}

/* Return the que topology for a Queue Group Index */
void
vrb2_qtopFromAcc(struct q_topology_t **qtop, int acc_enum,
		struct vrb2_conf *vrb2_conf)
{
	struct q_topology_t *p_qtop;
	p_qtop = NULL;
	switch (acc_enum) {
	case UL_4G:
		p_qtop = &(vrb2_conf->q_ul_4g);
		break;
	case UL_5G:
		p_qtop = &(vrb2_conf->q_ul_5g);
		break;
	case DL_4G:
		p_qtop = &(vrb2_conf->q_dl_4g);
		break;
	case DL_5G:
		p_qtop = &(vrb2_conf->q_dl_5g);
		break;
	case FFT:
		p_qtop = &(vrb2_conf->q_fft);
		break;
	case MLD:
		p_qtop = &(vrb2_conf->q_mld);
		break;
	default:
		/* NOTREACHED */
		LOG(ERR, "Unexpected error evaluating vrb2_qtopFromAcc %d", acc_enum);
		break;
	}
	*qtop = p_qtop;
}

/* Return the AQ depth for a Queue Group Index */
int
vrb2_aqDepth(int qg_idx, struct vrb2_conf *vrb2_conf)
{
#ifndef VRB2_STATIC_QMGR_ALLOCATION
	return VRB2_N0_DEPTH_LOG2;
#endif

	struct q_topology_t *q_top = NULL;
	int acc_enum = vrb2_accFromQgid(qg_idx, vrb2_conf);
	vrb2_qtopFromAcc(&q_top, acc_enum, vrb2_conf);
	if (q_top == NULL)
		return 1;
	return MAX(1, q_top->aq_depth_log2);
}

/* Return the AQ depth for a Queue Group Index */
int
vrb2_aqNum(int qg_idx, struct vrb2_conf *vrb2_conf)
{
	struct q_topology_t *q_top = NULL;
	int acc_enum = vrb2_accFromQgid(qg_idx, vrb2_conf);
	vrb2_qtopFromAcc(&q_top, acc_enum, vrb2_conf);
	if (q_top == NULL)
		return 0;
	return q_top->num_aqs_per_groups;
}

/* Register offset for QoS Registers */
static int
vrb2_qos_offset(int vf_idx, int xl)
{
	int register_offset = HWPfQosmonBCntrlReg - HWPfQosmonACntrlReg;
	return VF_OFFSET_QOS * vf_idx + xl * register_offset;
}

/* Register offset for QoS Registers */
static int
vrb2_qos_action(int idx, int xl)
{
	int register_offset = HWPfQosmonBCntrlReg - HWPfQosmonACntrlReg;
	return 4 * idx + xl * register_offset;
}

static int
vrb2_write_config(void *dev, void *mapaddr, struct vrb2_conf *vrb2_conf, const bool first_cfg)
{
	uint32_t value, address, status, pg_config;
	int qg_idx, template_idx, vf_idx, acc, xl, i, aq_reg, engine, eng_type;
	uint8_t *d = mapaddr;
	int rlim, alen, tmptimestamp, totalQgs, ret;
	int numQgs, numQqsAcc, numEngines;
	hw_device *accel_dev = (hw_device *)dev;
	bool pg_required;

	/* Check we are already out of PG */
	status = vrb2_reg_read(d, HWPfHiSectionPowerGatingAck);
	pg_required = (status == VRB2_PG_MASK_0);
	if (pg_required) {
		/* Clock gate sections that will be un-PG */
		vrb2_reg_write(d, HWPfHiClkGateHystReg, VRB2_CLK_DIS_A0);
		/* Un-PG required sections */
		vrb2_reg_write(d, HWPfHiSectionPowerGatingReq, VRB2_PG_MASK_1);
		status = vrb2_reg_read(d, HWPfHiSectionPowerGatingAck);
		if (status != VRB2_PG_MASK_1) {
			LOG(ERR, "Unexpected status %x %x", status, VRB2_PG_MASK_1);
			return -ENODEV;
		}
		vrb2_reg_write(d, HWPfHiSectionPowerGatingReq, VRB2_PG_MASK_3);
		status = vrb2_reg_read(d, HWPfHiSectionPowerGatingAck);
		if (status != VRB2_PG_MASK_3) {
			LOG(ERR, "Unexpected status %x %x", status, VRB2_PG_MASK_3);
			return -ENODEV;
		}

		/* Explicitly reset QoS and QMGR SRAM to avoid parity error false alarm. */
		for (xl = 0; xl < VRB2_NUM_QOS; xl++) {
			for (vf_idx = 0; vf_idx < VRB2_NUM_VFS; vf_idx++) {
				address = HWPfQosmonARemThres1Vf + vrb2_qos_offset(vf_idx, xl);
				vrb2_reg_fast_write(d, address, 0);
				address = HWPfQosmonAThres2Vf + vrb2_qos_offset(vf_idx, xl);
				vrb2_reg_fast_write(d, address, 0);
				address = HWPfQosmonAWeiFracVf + vrb2_qos_offset(vf_idx, xl);
				vrb2_reg_fast_write(d, address, 0);
				address = HWPfQosmonARrWeiVf + vrb2_qos_offset(vf_idx, xl);
				vrb2_reg_fast_write(d, address, 0);
			}
			for (i = 0; i < 8; i++) {
				address = HWPfQosmonAXaction + vrb2_qos_action(i, xl);
				vrb2_reg_fast_write(d, address, 0);
			}
		}
		for (qg_idx = 0; qg_idx < VRB2_NUM_QGRPS; qg_idx++) {
			for (vf_idx = 0; vf_idx < VRB2_NUM_VFS; vf_idx++) {
				address = HWPfQmgrVfBaseAddr + vf_idx * sizeof(uint32_t) +
						qg_idx * sizeof(uint32_t) * 64;
				vrb2_reg_fast_write(d, address, 0);
			}
		}
	}

	/* Check the version of HW is the expected one. */
	value = vrb2_reg_read(d, HwPfQmgrIrqDebug1);
	if (value != VRB2_B0_VALUE) {
		LOG(WARN, "Unexpected older version of HW used (%x %x) - Fall back to ES1 mode.",
				value, VRB2_B0_VALUE);
		b0_variant = false;
	} else {
		b0_variant = true;
	}

	if (b0_variant && pg_required) {
		/* Initialize Qmgr ARAM memory. */
		vrb2_reg_fast_write(d, HWPfAramControlStatus, VRB2_ARAM_CONTROL + 1);
		usleep(200);
		status = vrb2_reg_read(d, HWPfAramControlStatus);
		if (status & 0x4)
			LOG(ERR, "ARAM initialization not complete");
		vrb2_reg_fast_write(d, HWPfAramControlStatus, VRB2_ARAM_CONTROL);
	}

	/* Adjust PG on the device. */
	status = vrb2_reg_read(d, HWPfHiSectionPowerGatingAck);
	pg_config = (vrb2_conf->q_fft.num_qgroups == 0 ? 0x1 : 0) |
			(vrb2_conf->q_mld.num_qgroups == 0 ? 0x2 : 0) |
			(vrb2_conf->q_ul_4g.num_qgroups == 0 ? 0x4 : 0) |
			(vrb2_conf->q_ul_5g.num_qgroups == 0 ? 0x8 : 0);
	status = vrb2_reg_read(d, HWPfHiSectionPowerGatingAck);
	if (status != pg_config) {
		LOG(INFO, "Adjust PG on the device from %x to %x", status, pg_config);
		vrb2_reg_write(d, HWPfHiSectionPowerGatingReq, pg_config);
		status = vrb2_reg_read(d, HWPfHiSectionPowerGatingAck);
		if (status != pg_config) {
			LOG(ERR, "Unexpected PG update %x while expecting %x", status, pg_config);
			return -ENODEV;
		}
	}

	/* Enable clocks for all sections */
	vrb2_reg_write(d, HWPfHiClkGateHystReg, b0_variant ? VRB2_CLK_EN : VRB2_CLK_EN_A0);

	/* Explicitly releasing AXI as this may be stopped after PF FLR/BME */
	vrb2_reg_write(d, HWPfDmaAxiControl, 1);

	/* Set the fabric mode */
	vrb2_reg_write(d, HWPfFabricM2iBufferReg, VRB2_FABRIC_MODE);

	/* Set default descriptor signature */
	vrb2_reg_write(d, HWPfDmaDescriptorSignature, 0);

	/* Enable the Error Detection in DMA */
	vrb2_reg_write(d, HWPfDmaErrorDetectionEn, VRB2_CFG_DMA_ERROR);

	/* AXI Cache configuration */
	vrb2_reg_write(d, HWPfDmaAxcacheReg, VRB2_CFG_AXI_CACHE);

	/* AXI Response configuration */
	vrb2_reg_write(d, HWPfDmaCfgRrespBresp, 0x0);

	/* Core Hang Recovery */
	for (engine = 0; engine < 32; engine++) {
		vrb2_reg_write(d, HWPfDmaChrCtrl + engine * 0x10, VRB2_CHR_ENABLE);
		vrb2_reg_write(d, HWPfDmaChrCleanupTshld + engine * 0x10, VRB2_CHR_THOLD);
	}

	/* Default DMA Configuration (Qmgr Enabled) */
	vrb2_reg_write(d, HWPfDmaConfig0Reg, 0);
	vrb2_reg_write(d, HWPfDmaQmanenSelect, 0xFFFFFFFF);
	vrb2_reg_write(d, HWPfDmaQmanen, 0);

	/* Default RLIM/ALEN configuration */
	rlim = 0;
	alen = 3;
	tmptimestamp = 0;
	address = HWPfDmaConfig1Reg;
	value = (1 << 31) + (rlim << 8) + (tmptimestamp << 6) + alen;
	vrb2_reg_write(d, address, value);

	/* Configure DMA Qmanager addresses */
	address = HWPfDmaQmgrAddrReg;
	value = HWPfQmgrEgressQueuesTemplate;
	vrb2_reg_write(d, address, value);

	/* Enable time out counters */
	vrb2_reg_write(d, HWPfDmaCmplTmOutCnt, 0xFFFFFFFF);
	for (eng_type = 0; eng_type < VRB2_ENG_TYPE_NUM; eng_type++)
		vrb2_reg_write(d, HWPfDmaProcTmOutCnt + eng_type * 4, VRB2_PROC_TIMEOUT);
	vrb2_reg_write(d, HWPfDmaConfigPtoutOutEn, 0xFFFFFFFF);
	vrb2_reg_write(d, HWPfDmaStatusToutProcess, 0xFFFFFFFF);
	vrb2_reg_write(d, HWPfDmaConfigCtoutOutDataEn, 0xFFFFFFFF);
	vrb2_reg_write(d, HWPfDmaConfigCtoutOutDescrEn, 0xFFFFFFFF);
	vrb2_reg_write(d, HWPfDmaConfigUnexpComplDataEn, 0xFFFFFFFF);
	vrb2_reg_write(d, HWPfDmaConfigUnexpComplDescrEn, 0xFFFFFFFF);
	vrb2_reg_write(d, HWPfDmaConfigPtoutOutEn, 0xFFFFFFFF);

	vrb2_reg_write(d, HWPfQmgrAramWatchdogCount, VRB2_QMGR_ARAM_TIMEOUT);
	vrb2_reg_write(d, HWPfQmgrAramWatchdogCounterEn, 0xFFFFFFFF);
	vrb2_reg_write(d, HWPfQmgrAxiWatchdogCount, VRB2_QMGR_AXI_TIMEOUT);
	vrb2_reg_write(d, HWPfQmgrAxiWatchdogCounterEn, 0xFFFFFFFF);
	vrb2_reg_write(d, HWPfQmgrProcessWatchdogCount, VRB2_QMGR_TIMEOUT);
	vrb2_reg_write(d, HWPfQmgrProcessWatchdogCounterEn, 0xFFFFFFFF);
	vrb2_reg_write(d, HWPfDmaClusterHangThld, VRB2_CLUST_TIMEOUT);
	if (b0_variant)
		vrb2_reg_write(d, HWPfDmaClusterCtrl, 0x1);

	/* Performance tuning. */
	int coretype[VRB2_NUM_TMPL] = {FFT, FFT, FFT, MLD, UL_5G, MLD, UL_5G, UL_5G,
			UL_4G, UL_5G, UL_4G, UL_5G, UL_4G, UL_5G, UL_4G, UL_4G,
			VRB2_UNUSED, VRB2_UNUSED, VRB2_UNUSED, VRB2_UNUSED,
			VRB2_UNUSED, VRB2_UNUSED, VRB2_UNUSED, VRB2_UNUSED,
			DL_5G, DL_4G, DL_5G, DL_5G, VRB2_UNUSED, DL_4G, VRB2_UNUSED, DL_4G
			};
	for (i = 0; i < VRB2_NUM_TMPL; i++) {
		address = HWPfFabricI2mRouterCore0 + VRB2_FABRIC_OFFSET * i;
		switch (coretype[i]) {
		case UL_5G:
			vrb2_reg_fast_write(d, address, VRB2_UL_5G_ROUTER_SIZE);
			break;
		case VRB2_UNUSED:
			break;
		default:
			vrb2_reg_fast_write(d, address, VRB2_DEFAULT_ROUTER_SIZE);
			break;
		}
	}
	for (i = 0; i < 4; i++) {
		value = 0;
		address = HWPfFabricI2mRouterCoreWeight + VRB2_FABRIC_OFFSET * i;
		for (engine = 0; engine < (VRB2_NUM_TMPL / 4); engine++) {
			if (coretype[engine * 4 + i] == UL_5G)
			value |= 1 << (engine * 4);
		}
		vrb2_reg_fast_write(d, address, value);
	}
	vrb2_reg_fast_write(d, HWPfFabricI2mRouteDmaDataWeight, VRB2_DMA_WEIGHT);
	vrb2_reg_fast_write(d, HWPfDmaIbThreshold, VRB2_IB_THHOLD);
	vrb2_reg_fast_write(d, HWPfDmaStreamIbThreshold, VRB2_STREAM_IB_THHOLD);
	vrb2_reg_fast_write(d, HWPfDmaWeightedSwitchingSmall, VRB2_DMA_SWITCH_DEFAULT);
	vrb2_reg_fast_write(d, HWPfDmaWeightedSwitchingLarge, VRB2_DMA_SWITCH_DEFAULT);
	vrb2_reg_fast_write(d, HWPfDmaWeightedSwitchingStream, VRB2_DMA_SWITCH_STREAM);

	/* Force the TD cold register to valid default value. */
	for (i = 0; i < 5; i++)
		vrb2_reg_fast_write(d, HWPfFeculColdCtrlReg + i * 0x1000,
				(1 << 17) + (15 << 4) + 2);

	/* ===== Qmgr Configuration ===== */
	/* Configuration of the AQueue Depth QMGR_GRP_0_DEPTH_LOG2 for UL */
	totalQgs = vrb2_conf->q_ul_4g.num_qgroups + vrb2_conf->q_ul_5g.num_qgroups +
			vrb2_conf->q_dl_4g.num_qgroups + vrb2_conf->q_dl_5g.num_qgroups +
			vrb2_conf->q_fft.num_qgroups + vrb2_conf->q_mld.num_qgroups;

	LOG(INFO, "Queue Groups UL4G %d DL4G %d UL5G %d DL5G %d FFT %d MLD %d",
			vrb2_conf->q_ul_4g.num_qgroups, vrb2_conf->q_dl_4g.num_qgroups,
			vrb2_conf->q_ul_5g.num_qgroups, vrb2_conf->q_dl_5g.num_qgroups,
			vrb2_conf->q_fft.num_qgroups, vrb2_conf->q_mld.num_qgroups);

	for (qg_idx = 0; qg_idx < totalQgs; qg_idx++) {
		address = HWPfQmgrDepthLog2Grp + sizeof(uint32_t) * qg_idx;
		value = vrb2_aqDepth(qg_idx, vrb2_conf);
		vrb2_reg_fast_write(d, address, value);
		address = HWPfQmgrTholdGrp + sizeof(uint32_t) * qg_idx;
		value = (1 << 16) + (1 << (vrb2_aqDepth(qg_idx, vrb2_conf) - 1));
		vrb2_reg_fast_write(d, address, value);
		address = HWPfQmgrArbQDepthGrp + sizeof(uint32_t) * qg_idx;
		vrb2_reg_fast_write(d, address, VRB2_ARB_QDEPTH);
	}

	/* Template Priority in incremental order */
	for (template_idx = 0; template_idx < VRB2_NUM_TMPL; template_idx++) {
		address = HWPfQmgrGrpTmplateReg0Indx + VRB2_BYTES_IN_WORD * template_idx;
		value = VRB2_TMPL_PRI_0;
		vrb2_reg_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg1Indx + VRB2_BYTES_IN_WORD * template_idx;
		value = VRB2_TMPL_PRI_1;
		vrb2_reg_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg2Indx + VRB2_BYTES_IN_WORD * template_idx;
		value = VRB2_TMPL_PRI_2;
		vrb2_reg_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg3Indx + VRB2_BYTES_IN_WORD * template_idx;
		value = VRB2_TMPL_PRI_3;
		vrb2_reg_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg4Indx + VRB2_BYTES_IN_WORD * template_idx;
		value = VRB2_TMPL_PRI_4;
		vrb2_reg_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg5Indx + VRB2_BYTES_IN_WORD * template_idx;
		value = VRB2_TMPL_PRI_5;
		vrb2_reg_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg6Indx + VRB2_BYTES_IN_WORD * template_idx;
		value = VRB2_TMPL_PRI_6;
		vrb2_reg_write(d, address, value);
		address = HWPfQmgrGrpTmplateReg7Indx + VRB2_BYTES_IN_WORD * template_idx;
		value = VRB2_TMPL_PRI_7;
		vrb2_reg_write(d, address, value);
	}

	address = HWPfQmgrGrpPriority;
	value = VRB2_CFG_QMGR_HI_P;
	vrb2_reg_write(d, address, value);

	/* Default Qmgr ECC configuration. */
	vrb2_reg_write(d, HWPfQmgrAdrGenEcc, VRB2_QMGR_ECC);

	/* Template Configuration */
	for (template_idx = 0; template_idx < VRB2_NUM_TMPL; template_idx++) {
		value = 0;
		address = HWPfQmgrGrpTmplateEnRegIndx + VRB2_BYTES_IN_WORD * template_idx;
		vrb2_reg_write(d, address, value);
	}
	/* 4GUL */
	numQgs = vrb2_conf->q_ul_4g.num_qgroups;
	numQqsAcc = 0;
	value = 0;
	for (qg_idx = numQqsAcc; qg_idx < (numQgs + numQqsAcc); qg_idx++)
		value |= (1 << qg_idx);
	for (template_idx = VRB2_SIG_UL_4G; template_idx <= VRB2_SIG_UL_4G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateEnRegIndx + VRB2_BYTES_IN_WORD * template_idx;
		vrb2_reg_write(d, address, value);
	}
	/* 5GUL */
	numQqsAcc += numQgs;
	numQgs	= vrb2_conf->q_ul_5g.num_qgroups;
	value = 0;
	numEngines = 0;
	for (qg_idx = numQqsAcc; qg_idx < (numQgs + numQqsAcc); qg_idx++)
		value |= (1 << qg_idx);
	for (template_idx = VRB2_SIG_UL_5G; template_idx <= VRB2_SIG_UL_5G_LAST;
			template_idx++) {
		/* Check engine power-on status */
		address = HwPfFecUl5gIbDebug0Reg + VRB2_ENGINE_OFFSET * template_idx;
		status = (vrb2_reg_read(d, address) >> 4) & 0x7;
		address = HWPfQmgrGrpTmplateEnRegIndx + VRB2_BYTES_IN_WORD * template_idx;
		if (status == 1) {
			vrb2_reg_write(d, address, value);
			numEngines++;
		} else
			vrb2_reg_write(d, address, 0);
#if RTE_VRB2_SINGLE_FEC == 1
		value = 0;
#endif
	}
	/* 4GDL */
	numQqsAcc += numQgs;
	numQgs	= vrb2_conf->q_dl_4g.num_qgroups;
	value = 0;
	for (qg_idx = numQqsAcc; qg_idx < (numQgs + numQqsAcc); qg_idx++)
		value |= (1 << qg_idx);
	for (template_idx = VRB2_SIG_DL_4G; template_idx <= VRB2_SIG_DL_4G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateEnRegIndx + VRB2_BYTES_IN_WORD * template_idx;
		vrb2_reg_write(d, address, value);
#if RTE_VRB2_SINGLE_FEC == 1
			value = 0;
#endif
	}
	/* 5GDL */
	numQqsAcc += numQgs;
	numQgs	= vrb2_conf->q_dl_5g.num_qgroups;
	value = 0;
	for (qg_idx = numQqsAcc; qg_idx < (numQgs + numQqsAcc); qg_idx++)
		value |= (1 << qg_idx);
	for (template_idx = VRB2_SIG_DL_5G; template_idx <= VRB2_SIG_DL_5G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateEnRegIndx + VRB2_BYTES_IN_WORD * template_idx;
		vrb2_reg_write(d, address, value);
#if RTE_VRB2_SINGLE_FEC == 1
		value = 0;
#endif
	}
	/* FFT */
	numQqsAcc += numQgs;
	numQgs	= vrb2_conf->q_fft.num_qgroups;
	value = 0;
	for (qg_idx = numQqsAcc; qg_idx < (numQgs + numQqsAcc); qg_idx++)
		value |= (1 << qg_idx);
	for (template_idx = VRB2_SIG_FFT; template_idx <= VRB2_SIG_FFT_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateEnRegIndx + VRB2_BYTES_IN_WORD * template_idx;
		vrb2_reg_write(d, address, value);
#if RTE_VRB2_SINGLE_FEC == 1
		value = 0;
#endif
	}
	/* MLD */
	numQqsAcc += numQgs;
	numQgs	= vrb2_conf->q_mld.num_qgroups;
	value = 0;
	for (qg_idx = numQqsAcc; qg_idx < (numQgs + numQqsAcc); qg_idx++)
		value |= (1 << qg_idx);
	for (template_idx = VRB2_SIG_MLD; template_idx <= VRB2_SIG_MLD_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateEnRegIndx
				+ VRB2_BYTES_IN_WORD * template_idx;
		vrb2_reg_write(d, address, value);
#if RTE_VRB2_SINGLE_FEC == 1
		value = 0;
#endif
	}

	/* Queue Group Function mapping */
	int qman_func_id[8] = {0, 2, 1, 3, 4, 5, 0, 0};
	for (i = 0; i < 4; i++) {
		value = 0;
		for (qg_idx = 0; qg_idx < VRB2_NUM_QGRPS_PER_WORD; qg_idx++) {
			acc = vrb2_accFromQgid(qg_idx + i * VRB2_NUM_QGRPS_PER_WORD,
					vrb2_conf);
			value |= qman_func_id[acc] << (qg_idx * 4);
		}
		vrb2_reg_write(d, HWPfQmgrGrpFunction0 + i * VRB2_BYTES_IN_WORD, value);
	}

	/* Configuration of the Arbitration QGroup depth to 1 */
	for (qg_idx = 0; qg_idx < VRB2_NUM_QGRPS; qg_idx++) {
		address = HWPfQmgrArbQDepthGrp + VRB2_BYTES_IN_WORD * qg_idx;
		value = 0;
		vrb2_reg_write(d, address, value);
	}

#ifdef VRB2_STATIC_QMGR_ALLOCATION
	/* This pointer to ARAM (512kB) is shifted by 2 (4B per register) */
	uint32_t aram_address = 0;
	for (qg_idx = 0; qg_idx < totalQgs; qg_idx++) {
		for (vf_idx = 0; vf_idx < vrb2_conf->num_vf_bundles; vf_idx++) {
			address = HWPfQmgrVfBaseAddr + vf_idx
					* VRB2_BYTES_IN_WORD + qg_idx
					* VRB2_BYTES_IN_WORD * 64;
			value = aram_address;
			vrb2_reg_fast_write(d, address, value);
			/* Offset ARAM Address for next memory bank
			* - increment of 4B
			*/
			aram_address += vrb2_aqNum(qg_idx, vrb2_conf) *
					(1 << vrb2_aqDepth(qg_idx, vrb2_conf));
		}
	}
	if (aram_address > VRB2_WORDS_IN_ARAM_SIZE) {
		LOG(ERR, "ARAM Configuration not fitting %d %d",
				aram_address, VRB2_WORDS_IN_ARAM_SIZE);
		return -EINVAL;
	}
#else
	/* Dynamic Qmgr allocation */
	vrb2_reg_write(d, HWPfQmgrAramAllocEn, 1);
	vrb2_reg_write(d, HWPfQmgrAramAllocSetupN0, 0x1000);
	vrb2_reg_write(d, HWPfQmgrAramAllocSetupN1, 0);
	vrb2_reg_write(d, HWPfQmgrAramAllocSetupN2, 0);
	vrb2_reg_write(d, HWPfQmgrAramAllocSetupN3, 0);
	vrb2_reg_write(d, HWPfQmgrSoftReset, 1);
	vrb2_reg_write(d, HWPfQmgrSoftReset, 0);
#endif
	/* Default ARAM Control configuration */
	vrb2_reg_write(d, HWPfAramControlStatus, VRB2_ARAM_CONTROL);

	/* ==== HI Configuration ==== */

	/* No Info Ring/MSI by default */
	address = HWPfHiInfoRingIntWrEnRegPf;
	value = 0;
	vrb2_reg_write(d, address, value);
	address = HWPfHiCfgMsiIntWrEnRegPf;
	value = 0xFFFFFFFF;
	vrb2_reg_write(d, address, value);
	/* Prevent Block on Transmit Error */
	address = HWPfHiBlockTransmitOnErrorEn;
	value = VRB2_BLOCK_ON_TX;
	vrb2_reg_write(d, address, value);
	/* Prevents to drop MSI in case FIFO is full */
	address = HWPfHiMsiDropEnableReg;
	value = 0;
	vrb2_reg_write(d, address, value);
	/* Set the PF Mode register */
	address = HWPfHiPfMode;
	value = ((vrb2_conf->pf_mode_en) ? VRB2_PF_VAL : 0) | VRB2_QSTRIDE_MAP;
	vrb2_reg_write(d, address, value);
	/* Explicitly releasing AXI after PF Mode */
	vrb2_reg_write(d, HWPfDmaAxiControl, 1);

	/* QoS overflow init */
	value = 1;
	address = HWPfQosmonAEvalOverflow0;
	vrb2_reg_write(d, address, value);
	address = HWPfQosmonBEvalOverflow0;
	vrb2_reg_write(d, address, value);

	if (vrb2_conf->pf_mode_en)
		LOG(INFO, "Configuration in PF mode (unexpected: VF mode should ideally be used)");
	else
		LOG(INFO, "Configuration in VF mode");

	for (vf_idx = 0; vf_idx < vrb2_conf->num_vf_bundles; vf_idx++) {
		for (acc = 0; acc < VRB2_MON_NUMS; acc++) {
			address = HWPfPermonACntrlRegVf + 256 * vf_idx + acc * VRB2_MON_OFFSET;
			value = 0x1; /* Reset */
			vrb2_reg_write(d, address, value);
			address = HWPfPermonACntrlRegVf + 256 * vf_idx + acc * VRB2_MON_OFFSET;
			value = 0x2; /* Start */
			vrb2_reg_write(d, address, value);
		}
	}

	for (acc = 0; acc < VRB2_MON_NUMS; acc++) {
		address = HWPfPermonACbControlFec + acc * VRB2_MON_OFFSET;
		value = 0x1;
		vrb2_reg_write(d, address, value);
		value = 0x2;
		vrb2_reg_write(d, address, value);
	}

	vrb2_reg_write(d, HWPfPermonAControlBusMon, VRB2_BUSMON_RESET);
	vrb2_reg_write(d, HWPfPermonAConfigBusMon, (0 << 8) + 1); /* Group 0-1 */
	vrb2_reg_write(d, HWPfPermonASkipCountBusMon, 0);
	vrb2_reg_write(d, HWPfPermonAControlBusMon, VRB2_BUSMON_START);
	vrb2_reg_write(d, HWPfPermonCControlBusMon, VRB2_BUSMON_RESET);
	vrb2_reg_write(d, HWPfPermonCConfigBusMon, (2 << 8) + 1); /* Group 2 */
	vrb2_reg_write(d, HWPfPermonCSkipCountBusMon, 0);
	vrb2_reg_write(d, HWPfPermonCControlBusMon, VRB2_BUSMON_START);

	/* Default FFT configuration */
	if (vrb2_conf->q_fft.num_qgroups > 0) {
		for (template_idx = 0; template_idx < VRB2_FFT_NUM; template_idx++) {
			ret = vrb2_fft_reconfig(d, template_idx, first_cfg, accel_dev);
			if (ret < 0)
				return ret;
		}
	}

	/* Enabling AQueues through the Queue hierarchy*/
	unsigned int  en_bitmask[VRB2_AQ_REG_NUM];
	for (vf_idx = 0; vf_idx < VRB2_NUM_VFS; vf_idx++) {
		for (qg_idx = 0; qg_idx < VRB2_NUM_QGRPS; qg_idx++) {
			for (aq_reg = 0;  aq_reg < VRB2_AQ_REG_NUM; aq_reg++)
				en_bitmask[aq_reg] = 0;
			if (vf_idx < vrb2_conf->num_vf_bundles && qg_idx < totalQgs) {
				for (aq_reg = 0;  aq_reg < VRB2_AQ_REG_NUM; aq_reg++) {
					if (vrb2_aqNum(qg_idx, vrb2_conf) >= 16 * (aq_reg + 1))
						en_bitmask[aq_reg] = 0xFFFF;
					else if (vrb2_aqNum(qg_idx, vrb2_conf) <= 16 * aq_reg)
						en_bitmask[aq_reg] = 0x0;
					else
						en_bitmask[aq_reg] = (1 << (vrb2_aqNum(qg_idx,
								vrb2_conf) - aq_reg * 16)) - 1;
				}
			}
			for (aq_reg = 0; aq_reg < VRB2_AQ_REG_NUM; aq_reg++) {
				address = HWPfQmgrAqEnableVf + vf_idx * 16 + aq_reg * 4;
				value = (qg_idx << 16) + en_bitmask[aq_reg];
				vrb2_reg_fast_write(d, address, value);
			}
		}
	}

	LOG(INFO, "PF VRB2 configuration complete");
	return 0;
}

static int
vrb2_allocate_info_ring(void *dev)
{
	hw_device *accel_pci_dev = (hw_device *)dev;
	void *bar0addr = accel_pci_dev->bar0Addr;
	uint32_t i, *ring_data;

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

	vrb2_reg_write(bar0addr, HWPfHiInfoRingBaseHiRegPf,
		(uint32_t) (accel_pci_dev->info_ring_phys_addr >> 32));
	vrb2_reg_write(bar0addr, HWPfHiInfoRingBaseLoRegPf,
		(uint32_t) (accel_pci_dev->info_ring_phys_addr));
	vrb2_reg_write(bar0addr, HWPfHiInfoRingPointerRegPf, 0xF000);

	accel_pci_dev->info_ring_head =
			(vrb2_reg_read(bar0addr, HWPfHiInfoRingPointerRegPf) & 0xFFF) /
			sizeof(uint32_t);

	/* Clear explicitly content of Info Ring */
	if (accel_pci_dev->info_ring != NULL) {
		for (i = 0; i < VRB2_INFO_RING_NUM_ENTRIES; i++) {
			ring_data = ((uint32_t *) accel_pci_dev->info_ring) + i;
			*ring_data = 0;
		}
	}

	LOG(DEBUG, "info_ring_head: %d", accel_pci_dev->info_ring_head);

	return 0;
}

static void
vrb2_free_info_ring(void *dev)
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
vrb2_enable_ir_regs(uint8_t *bar0addr)
{
	/* Enabling all interrupt and IR */
	vrb2_reg_write(bar0addr, HWPfHiInfoRingIntWrEnRegPf, 0xFFFFFFFF);
	vrb2_reg_write(bar0addr, HWPfHiCfgMsiIntWrEnRegPf, 0xFFFFFFFF);
	vrb2_reg_write(bar0addr, HWPfHiInfoRingVf2pfHiWrEnReg, 0xFFFFFFFF);
	vrb2_reg_write(bar0addr, HWPfHiInfoRingVf2pfLoWrEnReg, 0xFFFFFFFF);
	vrb2_reg_write(bar0addr, HWPfHiCfgMsiVf2pfHighWrEnReg, 0xFFFFFFFF);
	vrb2_reg_write(bar0addr, HWPfHiCfgMsiVf2pfLoWrEnReg, 0xFFFFFFFF);
}

static void
vrb2_disable_ir_regs(uint8_t *bar0addr)
{
	/* Disable all interrupts and IR */
	vrb2_reg_write(bar0addr, HWPfHiInfoRingIntWrEnRegPf, 0x0);
	vrb2_reg_write(bar0addr, HWPfHiCfgMsiIntWrEnRegPf, 0xFFFFFFFF);
	vrb2_reg_write(bar0addr, HWPfHiInfoRingVf2pfLoWrEnReg, 0x0);
	vrb2_reg_write(bar0addr, HWPfHiInfoRingVf2pfHiWrEnReg, 0x0);
	vrb2_reg_write(bar0addr, HWPfHiCfgMsiVf2pfHighWrEnReg, 0xFFFFFFFF);
	vrb2_reg_write(bar0addr, HWPfHiCfgMsiVf2pfLoWrEnReg, 0xFFFFFFFF);
}

static void
vrb2_reset_info_ring(hw_device *accel_dev)
{
	uint32_t phys_high, phys_low;
	hw_device *accel_pci_dev = (hw_device *)accel_dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;

	/* Set back the Info Ring base addresses */
	phys_high = (uint32_t) (accel_pci_dev->info_ring_phys_addr >> 32);
	phys_low = (uint32_t) (accel_pci_dev->info_ring_phys_addr);
	vrb2_reg_write(bar0addr, HWPfHiInfoRingPointerRegPf, phys_high);
	vrb2_reg_write(bar0addr, HWPfHiInfoRingBaseLoRegPf, phys_low);

	/* Clean up the ring and adjust pointer */
	vrb2_reg_write(bar0addr, HWPfHiInfoRingPointerRegPf, 0xF000);
	accel_pci_dev->info_ring_head =
			(vrb2_reg_read(bar0addr, HWPfHiInfoRingPointerRegPf) & 0xFFF) /
			sizeof(uint32_t);
}

void vrb2_cluster_reset(void *dev)
{
	hw_device *accel_dev = (hw_device *)dev;
	void *bar0addr = accel_dev->bar0Addr;
	uint16_t template_idx;

	vrb2_disable_ir_regs(bar0addr);

	LOG(DEBUG, "VRB2 cluster reset");
	/* Disable Qmgr Template Configuration */
	for (template_idx = 0; template_idx < VRB2_NUM_TMPL; template_idx++) {
		vrb2_reg_write(bar0addr, HWPfQmgrGrpTmplateEnRegIndx +
				sizeof(uint32_t) * template_idx, 0);
	}
	/* Stop the DMA and wait for 100 usec */
	vrb2_reg_write(bar0addr, HWPfDmaSoftResetReg, 0x2);
	usleep(100);
	/* cluster reset */
	vrb2_reg_write(bar0addr, HWPfHiCoresHardResetReg, 0xFFFFFFFF);
	vrb2_reg_write(bar0addr, HWPfHi5GHardResetReg, 0xFFFFFFFF);
	vrb2_reg_write(bar0addr, HWPfHiHardResetReg, 0x3FF);
	/* wait for 10 usecs */
	usleep(10);
	vrb2_reset_info_ring(accel_dev);
}

int vrb2_enable_intr(void *dev)
{
	int ret;
	hw_device *accel_pci_dev = (hw_device *)dev;
	void *bar0addr = accel_pci_dev->bar0Addr;

	LOG(DEBUG, "Enable VRB2 interrupts");
	/* allocate info ring */
	ret = vrb2_allocate_info_ring(dev);
	if (ret < 0) {
		LOG(ERR, "vrb2allocate info ring failed");
		return -1;
	}

	vrb2_enable_ir_regs(bar0addr);

	LOG(DEBUG, "Done");
	return 0;
}

int
vrb2_disable_intr(void *dev)
{
	hw_device *accel_pci_dev = (hw_device *)dev;
	void *bar0addr = accel_pci_dev->bar0Addr;

	LOG(DEBUG, "Disable interrupts");

	vrb2_disable_ir_regs(bar0addr);

	vrb2_free_info_ring(dev);

	LOG(DEBUG, "Done");
	return 0;
}

int
vrb2_configure(void *dev, void *bar0addr, const char *cfg_filename, const bool first_cfg)
{
	struct vrb2_conf vrb2_conf;
	int ret;

	ret = vrb2_read_config_file(cfg_filename, &vrb2_conf);
	if (ret != 0) {
		LOG(ERR, "Error reading config file.");
		return -1;
	}

	ret = vrb2_write_config(dev, bar0addr, &vrb2_conf, first_cfg);
	if (ret != 0) {
		LOG(ERR, "Error writing configuration for VRB2.");
		return -1;
	}
	hw_device *accel_dev = (hw_device *)dev;
	accel_dev->numvfs = vrb2_conf.num_vf_bundles;
	return 0;
}

void vrb2_device_data(void *dev)
{
	uint32_t vf_idx, num, min, max;
	uint64_t avg;
	hw_device *accel_dev = (hw_device *)dev;
	uint8_t *bar0addr = accel_dev->bar0Addr;

	LOG_RESP(INFO, "Device Status:: %d VFs", accel_dev->numvfs);
	for (vf_idx = 0; vf_idx < accel_dev->numvfs; vf_idx++)
		LOG_RESP(INFO, "-  VF %d %s", vf_idx,
			 bb_acc_device_status_str(accel_dev->dev_status[vf_idx]));
	LOG_RESP(INFO, "5GUL counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonACountVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "5GUL counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonAKCntLoVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "5GUL counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonACbCountFec, VRB2_5GUL_ENGS, VRB2_PMON_OFF_2);
	LOG_RESP(INFO, "5GDL counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonBCountVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "5GDL counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonBKCntLoVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "5GDL counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonBCbCountFec, VRB2_5GDL_ENGS, VRB2_PMON_OFF_2);
	LOG_RESP(INFO, "4GUL counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonCCountVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "4GUL counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonCKCntLoVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "4GUL counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonCCbCountFec, VRB2_4GUL_ENGS, VRB2_PMON_OFF_2);
	LOG_RESP(INFO, "4GDL counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonDCountVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "4GDL counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonDKCntLoVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "4GDL counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonDCbCountFec, VRB2_4GDL_ENGS, VRB2_PMON_OFF_2);
	LOG_RESP(INFO, "FFT counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonECountVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "FFT counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonEKCntLoVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "FFT counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonECbCountFec, VRB2_FFT_ENGS, VRB2_PMON_OFF_2);
	LOG_RESP(INFO, "MLD counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonFCountVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "MLD counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonFKCntLoVf, accel_dev->numvfs, VRB2_PMON_OFF_1);
	LOG_RESP(INFO, "MLD counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonFCbCountFec, VRB2_MLD_ENGS, VRB2_PMON_OFF_2);
	/* Bus Monitor */
	vrb2_reg_write(bar0addr, HWPfPermonAControlBusMon, VRB2_BUSMON_STOP);
	min = vrb2_reg_read(bar0addr, HWPfPermonAMinLatBusMon);
	max = vrb2_reg_read(bar0addr, HWPfPermonAMaxLatBusMon);
	num = vrb2_reg_read(bar0addr, HWPfPermonATotalReqCntBusMon);
	if (num > 0) {
		avg = (vrb2_reg_read(bar0addr, HWPfPermonATotalLatLowBusMon) +
				vrb2_reg_read(bar0addr, HWPfPermonATotalLatUpperBusMon)
				* ((uint64_t)1 << 32)) / num;
		LOG_RESP(INFO, "Bus Monitor A (ns) Avg %u - Num %u - Min %d - Max %d",
				(uint32_t) avg, num, min, max);
	}
	vrb2_reg_write(bar0addr, HWPfPermonAControlBusMon, VRB2_BUSMON_RESET);
	vrb2_reg_write(bar0addr, HWPfPermonAControlBusMon, VRB2_BUSMON_START);
	vrb2_reg_write(bar0addr, HWPfPermonCControlBusMon, VRB2_BUSMON_STOP);
	min = vrb2_reg_read(bar0addr, HWPfPermonCMinLatBusMon);
	max = vrb2_reg_read(bar0addr, HWPfPermonCMaxLatBusMon);
	num = vrb2_reg_read(bar0addr, HWPfPermonCTotalReqCntBusMon);
	if (num > 0) {
		avg = (vrb2_reg_read(bar0addr, HWPfPermonCTotalLatLowBusMon) +
				vrb2_reg_read(bar0addr, HWPfPermonCTotalLatUpperBusMon)
				* ((uint64_t)1 << 32)) / num;
		LOG_RESP(INFO, "Bus Monitor C (ns) Avg %u - Num %u - Min %d - Max %d",
				(uint32_t) avg, num, min, max);
	}
	vrb2_reg_write(bar0addr, HWPfPermonCControlBusMon, VRB2_BUSMON_RESET);
	vrb2_reg_write(bar0addr, HWPfPermonCControlBusMon, VRB2_BUSMON_START);
}
