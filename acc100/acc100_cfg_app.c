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

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>

#include "cfg_reader.h"
#include "acc100_cfg_app.h"
#include "acc100_pf_enum.h"
#include "bb_acc.h"
#include "bb_acc_log.h"

static void
acc100_reg_write(uint8_t *mmio_base, uint32_t offset, uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
	usleep(1000);
}

static uint32_t
acc100_reg_read(uint8_t *mmio_base, uint32_t offset)
{

	void *reg_addr = mmio_base + offset;
	uint32_t ret = *((volatile uint32_t *)(reg_addr));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	ret = __bswap_32(ret);
#endif
	return ret;
}

/**
 * Retrieve ACC100 device configuration from file. The file's content must keep
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
 * @param acc100_conf
 *   Pointer to structure that will hold ACC100 configuration
 *
 * @return
 *   Zero on success, negative value on failure.
 */
static int
acc100_read_config_file(const char *arg_cfg_filename,
		struct acc100_conf *acc100_conf)
{
	bool unsafe_path = cfg_file_check_path_safety(arg_cfg_filename);
	if (unsafe_path == true) {
		LOG(ERR, "config file path \"%s\" is not safe",
				arg_cfg_filename);
		return -1;
	} else
		return acc100_parse_conf_file(arg_cfg_filename, acc100_conf);
}

enum {UL_4G = 0, UL_5G, DL_4G, DL_5G, NUM_ACC};

/* Return the accelerator enum for a Queue Group Index */
int
accFromQgid(int qg_idx, struct acc100_conf *acc100_conf)
{
	int accQg[ACC100_NUM_QGRPS];
	int NumQGroupsPerFn[NUM_ACC];
	int acc, qgIdx, qgIndex = 0;
	for (qgIdx = 0; qgIdx < ACC100_NUM_QGRPS; qgIdx++)
		accQg[qgIdx] = 0;
	NumQGroupsPerFn[UL_4G] = acc100_conf->q_ul_4g.num_qgroups;
	NumQGroupsPerFn[UL_5G] = acc100_conf->q_ul_5g.num_qgroups;
	NumQGroupsPerFn[DL_4G] = acc100_conf->q_dl_4g.num_qgroups;
	NumQGroupsPerFn[DL_5G] = acc100_conf->q_dl_5g.num_qgroups;
	for (acc = UL_4G;  acc < NUM_ACC; acc++)
		for (qgIdx = 0; qgIdx < NumQGroupsPerFn[acc]; qgIdx++)
			accQg[qgIndex++] = acc;
	acc = accQg[qg_idx];
	return acc;
}

/* Return the que topology for a Queue Group Index */
void
qtopFromAcc(struct q_topology_t **qtop, int acc_enum,
		struct acc100_conf *acc100_conf)
{
	struct q_topology_t *p_qtop;
	p_qtop = NULL;
	switch (acc_enum) {
	case UL_4G:
		p_qtop = &(acc100_conf->q_ul_4g);
		break;
	case UL_5G:
		p_qtop = &(acc100_conf->q_ul_5g);
		break;
	case DL_4G:
		p_qtop = &(acc100_conf->q_dl_4g);
		break;
	case DL_5G:
		p_qtop = &(acc100_conf->q_dl_5g);
		break;
	default:
		/* NOTREACHED */
		LOG(ERR, "Unexpected error evaluating qtopFromAcc");
		break;
	}
	*qtop = p_qtop;
}

/* Return the AQ depth for a Queue Group Index */
int
aqDepth(int qg_idx, struct acc100_conf *acc100_conf)
{
	struct q_topology_t *q_top = NULL;
	int acc_enum = accFromQgid(qg_idx, acc100_conf);
	qtopFromAcc(&q_top, acc_enum, acc100_conf);
	if (q_top == NULL)
		return 1;
	return MAX(1, q_top->aq_depth_log2);
}

/* Return the AQ depth for a Queue Group Index */
int
aqNum(int qg_idx, struct acc100_conf *acc100_conf)
{
	struct q_topology_t *q_top = NULL;
	int acc_enum = accFromQgid(qg_idx, acc100_conf);
	qtopFromAcc(&q_top, acc_enum, acc100_conf);
	if (q_top == NULL)
		return 0;
	return q_top->num_aqs_per_groups;
}

static int
acc100_write_config(void *dev, void *mapaddr, struct acc100_conf *acc100_conf)
{
	uint32_t payload, address, status;
	int qg_idx, template_idx, vf_idx, acc, xl, i, j;
	/* QMGR_ARAM - memory internal to ACC100 */
	uint32_t aram_address = 0;

	uint8_t *bar0addr = mapaddr;

	payload = acc100_reg_read(bar0addr, HwPfPcieGpexBridgeControl);
	bool firstCfg = (payload != ACC100_CFG_PCI_BRIDGE);
	if (payload == 0xFFFFFFFF) {
		LOG(ERR, "MMIO is not accessible causing UR error over PCIe");
		return 1;
	}

	/* PCIe Bridge configuration */
	acc100_reg_write(bar0addr, HwPfPcieGpexBridgeControl,
			ACC100_CFG_PCI_BRIDGE);
	for (i = 1; i < 17; i++)
		acc100_reg_write(bar0addr,
				HwPfPcieGpexAxiAddrMappingWindowPexBaseHigh
				+ i * 16, 0);

	/* Prevent blocking AXI read on BRESP for AXI Write */
	address = HwPfPcieGpexAxiPioControl;
	payload = ACC100_CFG_PCI_AXI;
	acc100_reg_write(bar0addr, address, payload);

	/* Clear Error Register */
	address = HWPfHiIosf2axiErrLogReg;
	payload = 0xFFFFFFFF;
	acc100_reg_write(bar0addr, address, payload);

	/* 5GDL PLL phase shift */
	address = HWPfChaDl5gPllPhshft0;
	payload = 1;
	acc100_reg_write(bar0addr, address, payload);

	/* Explicitly releasing AXI as this may be stopped after PF FLR/BME */
	address = HWPfDmaAxiControl;
	payload = 1;
	acc100_reg_write(bar0addr, address, payload);

	/* Enable granular dynamic clock gating */
	address = HWPfHiClkGateHystReg;
	payload = ACC100_CLOCK_GATING_EN;
	acc100_reg_write(bar0addr, address, payload);

	/* Set default descriptor signature */
	address = HWPfDmaDescriptorSignatuture;
	payload = 0;
	acc100_reg_write(bar0addr, address, payload);

	/* Enable the Error Detection in DMA */
	payload = ACC100_CFG_DMA_ERROR;
	address = HWPfDmaErrorDetectionEn;
	acc100_reg_write(bar0addr, address, payload);

	/* AXI Cache configuration */
	payload = ACC100_CFG_AXI_CACHE;
	address = HWPfDmaAxcacheReg;
	acc100_reg_write(bar0addr, address, payload);

	/* Adjust PCIe Lane adaptation */
	for (i = 0; i < ACC100_QUAD_NUMS; i++)
		for (j = 0; j < ACC100_LANES_PER_QUAD; j++)
			acc100_reg_write(bar0addr, HwPfPcieLnAdaptctrl + i * ACC100_PCIE_QUAD_OFFSET
					+ j * ACC100_PCIE_LANE_OFFSET, ACC100_ADAPT);

	/* Enable PCIe live adaptation */
	for (i = 0; i < ACC100_QUAD_NUMS; i++)
		acc100_reg_write(bar0addr, HwPfPciePcsEqControl +
				i * ACC100_PCIE_QUAD_OFFSET, ACC100_PCS_EQ);

	/* Default DMA Configuration (Qmgr Enabled) */
	address = HWPfDmaConfig0Reg;
	payload = 0;
	acc100_reg_write(bar0addr, address, payload);
	address = HWPfDmaQmanen;
	payload = 0;
	acc100_reg_write(bar0addr, address, payload);

	/* Default RLIM/ALEN configuration */
	address = HWPfDmaConfig1Reg;
	payload = (1 << 31) + (23 << 8) + (1 << 6) + 7;
	acc100_reg_write(bar0addr, address, payload);

	/* Configure DMA Qmanager addresses */
	address = HWPfDmaQmgrAddrReg;
	payload = HWPfQmgrEgressQueuesTemplate;
	acc100_reg_write(bar0addr, address, payload);

	/* Default Fabric Mode */
	address = HWPfFabricMode;
	payload = ACC100_FABRIC_MODE;
	acc100_reg_write(bar0addr, address, payload);

	/* ===== Qmgr Configuration ===== */
	/* Configuration of the AQueue Depth QMGR_GRP_0_DEPTH_LOG2 for UL */
	int totalQgs = acc100_conf->q_ul_4g.num_qgroups +
			acc100_conf->q_ul_5g.num_qgroups +
			acc100_conf->q_dl_4g.num_qgroups +
			acc100_conf->q_dl_5g.num_qgroups;
	for (qg_idx = 0; qg_idx < totalQgs; qg_idx++) {
		address = HWPfQmgrDepthLog2Grp +
		BYTES_IN_WORD * qg_idx;
		payload = aqDepth(qg_idx, acc100_conf);
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQmgrTholdGrp +
		BYTES_IN_WORD * qg_idx;
		payload = (1 << 16) + (1 << (aqDepth(qg_idx, acc100_conf) - 1));
		acc100_reg_write(bar0addr, address, payload);
	}
	LOG(INFO, "Queue Groups: %d 5GUL, %d 5GDL, %d 4GUL, %d 4GDL",
			acc100_conf->q_ul_5g.num_qgroups,
			acc100_conf->q_dl_5g.num_qgroups,
			acc100_conf->q_ul_4g.num_qgroups,
			acc100_conf->q_dl_4g.num_qgroups);

	/* Template Priority in incremental order */
	for (template_idx = 0; template_idx < ACC100_NUM_TMPL; template_idx++) {
		address = HWPfQmgrGrpTmplateReg0Indx + BYTES_IN_WORD * template_idx;
		payload = TMPL_PRI_0;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQmgrGrpTmplateReg1Indx + BYTES_IN_WORD * template_idx;
		payload = TMPL_PRI_1;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQmgrGrpTmplateReg2indx + BYTES_IN_WORD * template_idx;
		payload = TMPL_PRI_2;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQmgrGrpTmplateReg3Indx + BYTES_IN_WORD * template_idx;
		payload = TMPL_PRI_3;
		acc100_reg_write(bar0addr, address, payload);
	}

	address = HWPfQmgrGrpPriority;
	payload = ACC100_CFG_QMGR_HI_P;
	acc100_reg_write(bar0addr, address, payload);

	/* Template Configuration */
	for (template_idx = 0; template_idx < ACC100_NUM_TMPL; template_idx++) {
		payload = 0;
		address = HWPfQmgrGrpTmplateReg4Indx
				+ BYTES_IN_WORD*template_idx;
		acc100_reg_write(bar0addr, address, payload);
	}
	/* 4GUL */
	int numQgs = acc100_conf->q_ul_4g.num_qgroups;
	int numQqsAcc = 0;
	payload = 0;
	for (qg_idx = numQqsAcc; qg_idx < (numQgs + numQqsAcc); qg_idx++)
		payload |= (1 << qg_idx);
	for (template_idx = SIG_UL_4G; template_idx <= SIG_UL_4G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx
				+ BYTES_IN_WORD*template_idx;
		acc100_reg_write(bar0addr, address, payload);
	}
	/* 5GUL */
	numQqsAcc += numQgs;
	numQgs	= acc100_conf->q_ul_5g.num_qgroups;
	payload = 0;
	int numEngines = 0;
	for (qg_idx = numQqsAcc; qg_idx < (numQgs + numQqsAcc); qg_idx++)
		payload |= (1 << qg_idx);
	for (template_idx = SIG_UL_5G; template_idx <= SIG_UL_5G_LAST;
			template_idx++) {
		/* Check engine power-on status */
		address = HwPfFecUl5gIbDebugReg + 0x1000 * template_idx;
		status = (acc100_reg_read(bar0addr, address) >> 4) & 0xF;
		address = HWPfQmgrGrpTmplateReg4Indx
				+ BYTES_IN_WORD*template_idx;
		if (status == 1) {
			acc100_reg_write(bar0addr, address, payload);
			numEngines++;
		} else
			acc100_reg_write(bar0addr, address, 0);
		#if RTE_ACC100_SINGLE_FEC == 1
		payload = 0;
		#endif
	}
	LOG(DEBUG, "Number of 5GUL engines %d", numEngines);
	/* 4GDL */
	numQqsAcc += numQgs;
	numQgs	= acc100_conf->q_dl_4g.num_qgroups;
	payload = 0;
	for (qg_idx = numQqsAcc; qg_idx < (numQgs + numQqsAcc); qg_idx++)
		payload |= (1 << qg_idx);
	for (template_idx = SIG_DL_4G; template_idx <= SIG_DL_4G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx
				+ BYTES_IN_WORD*template_idx;
		acc100_reg_write(bar0addr, address, payload);
		#if RTE_ACC100_SINGLE_FEC == 1
			payload = 0;
		#endif
	}
	/* 5GDL */
	numQqsAcc += numQgs;
	numQgs	= acc100_conf->q_dl_5g.num_qgroups;
	payload = 0;
	for (qg_idx = numQqsAcc; qg_idx < (numQgs + numQqsAcc); qg_idx++)
		payload |= (1 << qg_idx);
	for (template_idx = SIG_DL_5G; template_idx <= SIG_DL_5G_LAST;
			template_idx++) {
		address = HWPfQmgrGrpTmplateReg4Indx
				+ BYTES_IN_WORD*template_idx;
		acc100_reg_write(bar0addr, address, payload);
		#if RTE_ACC100_SINGLE_FEC == 1
		payload = 0;
		#endif
	}

	/* Queue Group Function mapping */
	int qman_func_id[8] = {0, 2, 1, 3, 4, 0, 0, 0};
	address = HWPfQmgrGrpFunction0;
	payload = 0;
	for (qg_idx = 0; qg_idx < 8; qg_idx++) {
		acc = accFromQgid(qg_idx, acc100_conf);
		payload |= qman_func_id[acc]<<(qg_idx * 4);
	}
	acc100_reg_write(bar0addr, address, payload);

	/* Configuration of the Arbitration QGroup depth to 1 */
	for (qg_idx = 0; qg_idx < totalQgs; qg_idx++) {
		address = HWPfQmgrArbQDepthGrp +
		BYTES_IN_WORD * qg_idx;
		payload = 0;
		acc100_reg_write(bar0addr, address, payload);
	}

	/* This pointer to ARAM (128kB) is shifted by 2 (4B per register) */
	for (qg_idx = 0; qg_idx < totalQgs; qg_idx++) {
		for (vf_idx = 0; vf_idx < acc100_conf->num_vf_bundles;
				vf_idx++) {
			address = HWPfQmgrVfBaseAddr + vf_idx
					* BYTES_IN_WORD + qg_idx
					* BYTES_IN_WORD
					* ACC100_QMGR_BA_STRIDE;
			payload = aram_address;
			acc100_reg_write(bar0addr, address, payload);
			/* Offset ARAM Address for next memory bank
			 * - increment of 4B
			 */
			aram_address += aqNum(qg_idx, acc100_conf) *
					(1 << aqDepth(qg_idx, acc100_conf));
		}
	}

	if (aram_address > (WORDS_IN_ARAM_SIZE)) {
		LOG(ERR, "ARAM Configuration not fitting into 128kB");
		return -EINVAL;
	}

	/* ==== HI Configuration ==== */

	/* No Info Ring/MSI by default */
	acc100_reg_write(bar0addr, HWPfHiInfoRingIntWrEnRegPf, 0);
	acc100_reg_write(bar0addr, HWPfHiInfoRingVf2pfLoWrEnReg, 0);
	acc100_reg_write(bar0addr, HWPfHiCfgMsiIntWrEnRegPf, 0xFFFFFFFF);
	acc100_reg_write(bar0addr, HWPfHiCfgMsiVf2pfLoWrEnReg, 0xFFFFFFFF);

	/* Prevent Block on Transmit Error */
	address = HWPfHiBlockTransmitOnErrorEn;
	payload = 0;
	acc100_reg_write(bar0addr, address, payload);
	/* Prevents to drop MSI */
	address = HWPfHiMsiDropEnableReg;
	payload = 0;
	acc100_reg_write(bar0addr, address, payload);

	/* Set the PF Mode register */
	address = HWPfHiPfMode;
	payload = (acc100_conf->pf_mode_en) ? 2 : 0;
	acc100_reg_write(bar0addr, address, payload);
	if (acc100_conf->pf_mode_en)
		LOG(INFO, "Configuration in PF mode (unexpected: VF mode should ideally be used)");
	else
		LOG(INFO, "Configuration in VF mode");

	/* QoS overflow init */
	payload = 1;
	address = HWPfQosmonAEvalOverflow0;
	acc100_reg_write(bar0addr, address, payload);
	address = HWPfQosmonBEvalOverflow0;
	acc100_reg_write(bar0addr, address, payload);

	/* Enable PMon and QosMon */
	address = HWPfDmaPmEnable;
	payload = 0x1;
	acc100_reg_write(bar0addr, address, payload);
	address = HWPfDmaQosEnable;
	payload = 0x1;
	acc100_reg_write(bar0addr, address, payload);

	for (vf_idx = 0; vf_idx < acc100_conf->num_vf_bundles; vf_idx++) {
		address = HWPfPermonACntrlRegVf + 256 * vf_idx;
		payload = 0x1; /* Reset */
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfPermonACntrlRegVf + 256 * vf_idx;
		payload = 0x2; /* Start */
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfPermonBCntrlRegVf + 256 * vf_idx;
		payload = 0x1; /* Reset */
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfPermonBCntrlRegVf + 256 * vf_idx;
		payload = 0x2; /* Start */
		acc100_reg_write(bar0addr, address, payload);
	}

	address = HWPfPermonACbControlFec;
	payload = 0x1;
	acc100_reg_write(bar0addr, address, payload);
	payload = 0x2;
	acc100_reg_write(bar0addr, address, payload);
	address = HWPfPermonBCbControlFec;
	payload = 0x1;
	acc100_reg_write(bar0addr, address, payload);
	payload = 0x2;
	acc100_reg_write(bar0addr, address, payload);


	/* ==== QoS Configuration ==== */
	/* Loop for both clusters */
	int register_offset = HWPfQosmonBCntrlReg
			- HWPfQosmonACntrlReg;

	for (xl = 0; xl < 2; xl++) {
		/* Enable+Clear QosMon and Block Size granularity to 2^3 */
		address = HWPfQosmonACntrlReg + xl
				* register_offset;
		payload = (1 << 8) + 1; /* Enable + Reset */
		acc100_reg_write(bar0addr, address, payload);
	}
	usleep(1000);

	address = HWPfDmaQosEnable;
	payload = 1;
	acc100_reg_write(bar0addr, address, payload);

	for (xl = 0; xl < 2; xl++) {
		/* Enable+Clear QosMon and Block Size granularity to 2^3 */
		address = HWPfQosmonACntrlReg + xl
				* register_offset;
		payload = (1 << 9) + (1 << 4) + (3 << 1);
		acc100_reg_write(bar0addr, address, payload);
	}

	for (xl = 0; xl < 2; xl++) {
		/* Clock Divider */
		int clock_divider = 5;
		address = HWPfQosmonADivTerm + xl
				* register_offset;
		payload = (clock_divider - 1);
		acc100_reg_write(bar0addr, address, payload);
		/* Terminal Counter resolution - 1us */
		address = HWPfQosmonATickTerm + xl
				* register_offset;
		payload = 400 / clock_divider - 1;
		acc100_reg_write(bar0addr, address, payload);
		/* Evaluation Window - 5us */
		address = HWPfQosmonAEvalTerm + xl
				* register_offset;
		payload = 5 - 1;
		acc100_reg_write(bar0addr, address, payload);
		/* Average Window - 500ms */
		address = HWPfQosmonAAveTerm + xl
				* register_offset;
		payload = 100 - 1;
		acc100_reg_write(bar0addr, address, payload);
	}

	for (vf_idx = 0; vf_idx < acc100_conf->num_vf_bundles; vf_idx++) {
		/* UL */
		int gbr = acc100_conf->arb_ul_5g[0].gbr_threshold1;
		gbr = 128000;
		if (gbr == 0)
			gbr = 1;
		address = HWPfQosmonARemThres1Vf +
				VF_OFFSET_QOS * vf_idx;
		payload = gbr;

		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQosmonAThres2Vf +
				VF_OFFSET_QOS * vf_idx;
		payload = 0;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQosmonAWeiFracVf +
				VF_OFFSET_QOS * vf_idx;
		payload = ((acc100_conf->arb_ul_5g[0].round_robin_weight + 1)
				<< 24) / gbr;
		acc100_reg_write(bar0addr, address, payload);

		/* DL */
		gbr = acc100_conf->arb_dl_5g[0].gbr_threshold1;
		if (gbr == 0)
			gbr = 1;
		address = HWPfQosmonBRemThres1Vf +
				VF_OFFSET_QOS * vf_idx;
		payload = gbr;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQosmonBThres2Vf +
				VF_OFFSET_QOS * vf_idx;
		payload = gbr / 2;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQosmonBWeiFracVf +
				VF_OFFSET_QOS * vf_idx;
		payload = ((acc100_conf->arb_dl_5g[0].round_robin_weight + 1)
				<< 24) / gbr;
		acc100_reg_write(bar0addr, address, payload);
	}
	/* Explicitly reset remaining bundles parameters */
	for (vf_idx = acc100_conf->num_vf_bundles;
			vf_idx < ACC100_NUM_VFS; vf_idx++) {
		payload = 0;
		address = HWPfQosmonARemThres1Vf +
				VF_OFFSET_QOS * vf_idx;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQosmonAThres2Vf +
				VF_OFFSET_QOS * vf_idx;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQosmonAWeiFracVf +
				VF_OFFSET_QOS * vf_idx;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQosmonBRemThres1Vf +
				VF_OFFSET_QOS * vf_idx;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQosmonBThres2Vf +
				VF_OFFSET_QOS * vf_idx;
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQosmonBWeiFracVf +
				VF_OFFSET_QOS * vf_idx;
		acc100_reg_write(bar0addr, address, payload);
	}

	/* QoS LDPC Configuration */
	for (xl = 0; xl < 4; xl++) {
		/* Disable feature for now*/
		int qos_iterB = 20;
		int qos_iterA = 10;
		int qos_fracStep = 10;
		int qos_fracShift = 10;
		int qos_threshold = 10;
		address = HWPfQosmonAIterationConfig0Low
				+ 8 * xl;
		payload = qos_fracShift + (qos_fracStep << 4);
		acc100_reg_write(bar0addr, address, payload);
		address = HWPfQosmonAIterationConfig0High
				+ 8 * xl;
		payload = qos_iterB + (qos_iterA << 6)
				+ (qos_threshold << 12);
		acc100_reg_write(bar0addr, address, payload);
	}

	/* Enable Qos Scaling */
	address = HWPfDmaQosScale;
	payload = 0x3;
	acc100_reg_write(bar0addr, address, payload);

	for (xl = 0; xl < 2; xl++) {
		/* Enable QosMon with block Size granularity to 2^3 */
		address = HWPfQosmonACntrlReg + xl
				* register_offset;
		payload = (1 << 8) + (1 << 4) + (3 << 1);
		acc100_reg_write(bar0addr, address, payload);
	}

	/* HARQ DDR Configuration */
	unsigned int ddrSizeInMb;
	if (acc100_conf->num_vf_bundles == 1)
		ddrSizeInMb = ACC100_HARQ_TOTAL_DDR;
	else if (acc100_conf->num_vf_bundles == 2)
		ddrSizeInMb = ACC100_HARQ_TOTAL_DDR >> 1;
	else if (acc100_conf->num_vf_bundles <= 4)
		ddrSizeInMb = ACC100_HARQ_TOTAL_DDR >> 2;
	else
		ddrSizeInMb = ACC100_HARQ_TOTAL_DDR >> 4;
	for (vf_idx = 0; vf_idx < acc100_conf->num_vf_bundles; vf_idx++) {
		address = HWPfDmaVfDdrBaseRw + vf_idx
				* 0x10;
		payload = ((vf_idx * (ddrSizeInMb / 64)) << 16) +
				(ddrSizeInMb - 1);
		acc100_reg_write(bar0addr, address, payload);
	}

	/* Report SKU version of the device */
	payload = acc100_reg_read(bar0addr, HwPfPcieRomVersion);
	if (payload == ACC100_ROM_VER_SKU_A)
		LOG(INFO, " ROM version MM 99AD92");
	else if (payload == ACC100_ROM_VER_SKU_B)
		LOG(INFO, " ROM version MM 99ANA5");
	else
		LOG(INFO, " ROM version -undefined-");

	uint32_t version = 0;
	for (i = 0; i < 4; i++)
		version += acc100_reg_read(bar0addr,
				HWPfDdrPhyIdtmFwVersion + 4 * i) << (8 * i);
	if (version != ACC100_PRQ_DDR_VER) {
		LOG(WARN, "* Note: Not on DDR PRQ version %8x != %08x",
				version, ACC100_PRQ_DDR_VER);
	} else if (firstCfg) {
		/* ---- DDR configuration at boot up --- */
		/* Read Clear Ddr training status */
		acc100_reg_read(bar0addr, HWPfChaDdrStDoneStatus);
		/* Reset PHY/IDTM/UMMC */
		acc100_reg_write(bar0addr, HWPfChaDdrWbRstCfg, 3);
		acc100_reg_write(bar0addr, HWPfChaDdrApbRstCfg, 2);
		acc100_reg_write(bar0addr, HWPfChaDdrPhyRstCfg, 2);
		acc100_reg_write(bar0addr, HWPfChaDdrCpuRstCfg, 3);
		acc100_reg_write(bar0addr, HWPfChaDdrSifRstCfg, 2);
		usleep(ACC100_MS_IN_US);
		/* Reset WB and APB resets */
		acc100_reg_write(bar0addr, HWPfChaDdrWbRstCfg, 2);
		acc100_reg_write(bar0addr, HWPfChaDdrApbRstCfg, 3);
		/* Configure PHY-IDTM */
		acc100_reg_write(bar0addr, HWPfDdrPhyIdletimeout, 0x3e8);
		/* IDTM timing registers */
		acc100_reg_write(bar0addr, HWPfDdrPhyRdLatency, 0x13);
		acc100_reg_write(bar0addr, HWPfDdrPhyRdLatencyDbi, 0x15);
		acc100_reg_write(bar0addr, HWPfDdrPhyWrLatency, 0x10011);
		/* Configure SDRAM MRS registers */
		acc100_reg_write(bar0addr, HWPfDdrPhyMr01Dimm, 0x3030b70);
		acc100_reg_write(bar0addr, HWPfDdrPhyMr01DimmDbi, 0x3030b50);
		acc100_reg_write(bar0addr, HWPfDdrPhyMr23Dimm, 0x30);
		acc100_reg_write(bar0addr, HWPfDdrPhyMr67Dimm, 0xc00);
		acc100_reg_write(bar0addr, HWPfDdrPhyMr45Dimm, 0x4000000);
		/* Configure active lanes */
		acc100_reg_write(bar0addr, HWPfDdrPhyDqsCountMax, 0x9);
		acc100_reg_write(bar0addr, HWPfDdrPhyDqsCountNum, 0x9);
		/* Configure WR/RD leveling timing registers */
		acc100_reg_write(bar0addr, HWPfDdrPhyWrlvlWwRdlvlRr, 0x101212);
		/* Configure what trainings to execute */
		acc100_reg_write(bar0addr, HWPfDdrPhyTrngType, 0x2d3c);
		/* Releasing PHY reset */
		acc100_reg_write(bar0addr, HWPfChaDdrPhyRstCfg, 3);
		/* Configure Memory Controller registers */
		acc100_reg_write(bar0addr, HWPfDdrMemInitPhyTrng0, 0x3);
		/* Adjust default ROW address to end of 4GB area range */
		acc100_reg_write(bar0addr, HWPfDdrPhyDramRow, 0xffff);
		acc100_reg_write(bar0addr, HWPfDdrBcDram, 0x3c232003);
		acc100_reg_write(bar0addr, HWPfDdrBcAddrMap, 0x31);
		/* Configure UMMC BC timing registers */
		acc100_reg_write(bar0addr, HWPfDdrBcRef, 0xa22);
		acc100_reg_write(bar0addr, HWPfDdrBcTim0, 0x4050501);
		acc100_reg_write(bar0addr, HWPfDdrBcTim1, 0xf0b0476);
		acc100_reg_write(bar0addr, HWPfDdrBcTim2, 0x103);
		acc100_reg_write(bar0addr, HWPfDdrBcTim3, 0x144050a1);
		acc100_reg_write(bar0addr, HWPfDdrBcTim4, 0x23300);
		acc100_reg_write(bar0addr, HWPfDdrBcTim5, 0x4230276);
		acc100_reg_write(bar0addr, HWPfDdrBcTim6, 0x857914);
		acc100_reg_write(bar0addr, HWPfDdrBcTim7, 0x79100232);
		acc100_reg_write(bar0addr, HWPfDdrBcTim8, 0x100007ce);
		acc100_reg_write(bar0addr, HWPfDdrBcTim9, 0x50020);
		acc100_reg_write(bar0addr, HWPfDdrBcTim10, 0x40ee);
		/* Configure UMMC DFI timing registers */
		acc100_reg_write(bar0addr, HWPfDdrDfiInit, 0x5000);
		acc100_reg_write(bar0addr, HWPfDdrDfiTim0, 0x15030006);
		acc100_reg_write(bar0addr, HWPfDdrDfiTim1, 0x11305);
		acc100_reg_write(bar0addr, HWPfDdrDfiPhyUpdEn, 0x1);
		acc100_reg_write(bar0addr, HWPfDdrUmmcIntEn, 0x1f);
		/* Release IDTM CPU out of reset */
		acc100_reg_write(bar0addr, HWPfChaDdrCpuRstCfg, 0x2);
		/* Wait PHY-IDTM to finish static training */
		for (i = 0; i < ACC100_DDR_TRAINING_MAX; i++) {
			usleep(ACC100_MS_IN_US);
			payload = acc100_reg_read(bar0addr,
					HWPfChaDdrStDoneStatus);
			if (payload & 1)
				break;
		}
		LOG(INFO, "DDR Training completed in %d ms", i);
		/* Enable Memory Controller */
		acc100_reg_write(bar0addr, HWPfDdrUmmcCtrl, 0x401);
		/* Release AXI interface reset */
		acc100_reg_write(bar0addr, HWPfChaDdrSifRstCfg, 3);
	}

	/* Enabling AQueues through the Queue hierarchy*/
	for (vf_idx = 0; vf_idx < ACC100_NUM_VFS; vf_idx++) {
		for (qg_idx = 0; qg_idx < ACC100_NUM_QGRPS; qg_idx++) {
			payload = 0;
			if (vf_idx < acc100_conf->num_vf_bundles &&
					qg_idx < totalQgs)
				payload = (1 << aqNum(qg_idx, acc100_conf)) - 1;
			address = HWPfQmgrAqEnableVf
					+ vf_idx * BYTES_IN_WORD;
			payload += (qg_idx << 16);
			acc100_reg_write(bar0addr, address, payload);
		}
	}

	/* Manage exception after a partial soft reboot on some platforms. */
	if (acc100_reg_read(bar0addr, HWPfDmaStatusDmaHwErr) == 1) {
		payload = acc100_reg_read(bar0addr, HWPfDmaStatusDmaHwErr);
		status  = acc100_reg_read(bar0addr, HWPfHiIosf2axiErrLogReg);
		LOG(ERR, "The device unexpectedly failed to initialize correctly (Error info 0x%08x 0x%08x).",
				payload, status);
		LOG(ERR, "PF FLR and reconfiguration of the device is required.");
		return -1;
	}

	LOG(INFO, "PF ACC100 configuration complete");
	return 0;
}

int
acc100_configure(void *dev, void *bar0addr, const char *cfg_filename, const bool first_cfg)
{
	struct acc100_conf acc100_conf;
	int ret;

	ret = acc100_read_config_file(cfg_filename, &acc100_conf);
	if (ret != 0) {
		LOG(ERR, "Error reading config file");
		return -1;
	}

	ret = acc100_write_config(dev, bar0addr, &acc100_conf);
	if (ret != 0) {
		LOG(ERR, "Error writing configuration for ACC100");
		return -1;
	}
	hw_device *accel_dev = (hw_device *)dev;
	accel_dev->numvfs = acc100_conf.num_vf_bundles;

	return 0;
}

void acc100_device_data(void *dev)
{
	uint32_t vf_idx;

	hw_device *accel_dev = (hw_device *)dev;

	LOG_RESP(INFO, "Device Status:: %d VFs", accel_dev->numvfs);
	for (vf_idx = 0; vf_idx < accel_dev->numvfs; vf_idx++)
		LOG_RESP(INFO, "-  VF %d %s", vf_idx,
			 bb_acc_device_status_str(accel_dev->dev_status[vf_idx]));
	LOG_RESP(INFO, "5GUL counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonACountVf, accel_dev->numvfs, ACC100_PMON_OFF_1);
	LOG_RESP(INFO, "5GUL counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonAKCntLoVf, accel_dev->numvfs, ACC100_PMON_OFF_1);
	LOG_RESP(INFO, "5GUL counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonACbCountFec, ACC100_5GUL_ENGS, ACC100_PMON_OFF_2);
	LOG_RESP(INFO, "5GDL counters: Code Blocks");
	print_all_stat32(accel_dev, HWPfPermonBCountVf, accel_dev->numvfs, ACC100_PMON_OFF_1);
	LOG_RESP(INFO, "5GDL counters: Data (Bytes)");
	print_all_stat32(accel_dev, HWPfPermonBKCntLoVf, accel_dev->numvfs, ACC100_PMON_OFF_1);
	LOG_RESP(INFO, "5GDL counters: Per Engine");
	print_all_stat32(accel_dev, HWPfPermonBCbCountFec, ACC100_5GDL_ENGS, ACC100_PMON_OFF_2);
}

