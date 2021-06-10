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

#ifndef _ACC100_CFG_APP_H_
#define _ACC100_CFG_APP_H_

#define ACC100_CONFIG_FILE_ENV "ACC100_CONFIG_FILE"
#define ACC100_CONFIG_FILE_NAME "acc100_config.cfg"

#define ACC100_DDR_ECC_ENABLE

#define BYTES_IN_WORD 4
#define WORDS_IN_ARAM_SIZE (128 * 1024 / 4)

#define ACC100_NUM_QGRPS 8
#define ACC100_NUM_TMPL  32
#define VF_OFFSET_QOS 16 /* offset in Memory Space specific to QoS Mon */
#define TMPL_PRI_0 0x03020100
#define TMPL_PRI_1 0x07060504
#define TMPL_PRI_2 0x0b0a0908
#define TMPL_PRI_3 0x0f0e0d0c

#define SIG_UL_5G      0
#define SIG_UL_5G_LAST 7
#define SIG_DL_5G      13
#define SIG_DL_5G_LAST 15
#define SIG_UL_4G      16
#define SIG_UL_4G_LAST 21
#define SIG_DL_4G      27
#define SIG_DL_4G_LAST 31

#define ACC100_NUM_VFS 16 /**< Number of Virtual Functions ACC100 supports */
#define ACC100_QMGR_BA_STRIDE 64  /**< Base address stride for Qmgr */

/* ACC100 Configuration */
#define ACC100_CFG_DDR_ECC_EN 0x842304
#define ACC100_CFG_DDR_ECC_DIS 0x842300
#define ACC100_CFG_DMA_ERROR 0x3D7
#define ACC100_CFG_AXI_CACHE 0x11
#define ACC100_CFG_QMGR_HI_P 0x0F0F
#define ACC100_CFG_PCI_AXI 0xC003
#define ACC100_CFG_PCI_BRIDGE 0x40006033
#define ACC100_QUAD_NUMS 4
#define ACC100_PCIE_QUAD_OFFSET 0x2000
#define ACC100_PCS_EQ 0x6007
#define ACC100_CLOCK_GATING_EN  0x30000
/* DDR Size to be split across VFs */
#define ACC100_HARQ_TOTAL_DDR   (4096)
#define ACC100_PRQ_DDR_VER       0x10092020
#define ACC100_MS_IN_US         (1000)
#define ACC100_DDR_TRAINING_MAX (5000)
#define ACC100_FABRIC_MODE      0xB

/**
 * Definition of Queue Topology for ACC100 Configuration
 * Some level of details is abstracted out to expose a clean interface
 * given that comprehensive flexibility is not required
 */
struct q_topology_t {
	/** Number of QGroups for DL/UL in incremental order of priority */
	uint16_t num_qgroups;
	/**
	 * All QGroups have the same number of AQs here.
	 * Note : Could be made a 16-array if more flexibility is really
	 * required
	 */
	uint16_t num_aqs_per_groups;
	/**
	 * Depth of the AQs is the same of all QGroups here. Log2 Enum : 2^N
	 * Note : Could be made a 16-array if more flexibility is really
	 * required
	 */
	uint16_t aq_depth_log2;
};

/**
 * Definition of Arbitration related parameters for ACC100 Configuration for UL
 * or DL
 */
struct arbitration_t {
	/** Default Weight for VF Fairness Arbitration */
	uint16_t round_robin_weight;
	uint32_t gbr_threshold1;    /**< Guaranteed Bitrate Threshold 1 */
	uint32_t gbr_threshold2;    /**< Guaranteed Bitrate Threshold 2 */
};

/**
 * Structure to pass ACC100 configuration.
 * Note: all VF Bundles will have the same configuration.
 */
struct acc100_conf {
	bool pf_mode_en;  /**< 1 if PF is used for dataplane, 0 for VFs */
	/** 1 if input '1' bit is represented by a positive LLR value, 0 if '1'
	 * bit is represented by a negative value.
	 */
	bool input_pos_llr_1_bit;
	/** 1 if output '1' bit is represented by a positive value, 0 if '1'
	 * bit is represented by a negative value.
	 */
	bool output_pos_llr_1_bit;
	uint16_t num_vf_bundles;  /**< Number of VF bundles to setup */
	struct q_topology_t q_ul_4g;  /**< Uplink queues */
	struct q_topology_t q_dl_4g;  /**< Downlink queues */
	struct q_topology_t q_ul_5g;  /**< Uplink queues */
	struct q_topology_t q_dl_5g;  /**< Downlink queues */
	/** Uplink arbitration configuration */
	struct arbitration_t arb_ul_4g[ACC100_NUM_VFS];
	/** Downlink arbitration configuration */
	struct arbitration_t arb_dl_4g[ACC100_NUM_VFS];
	/** Uplink arbitration configuration */
	struct arbitration_t arb_ul_5g[ACC100_NUM_VFS];
	/** Downlink arbitration configuration */
	struct arbitration_t arb_dl_5g[ACC100_NUM_VFS];
};

/*
 * Configure ACC100
 */
int
acc100_configure(void *bar0addr, const char *arg_cfg_filename);

int
acc100_parse_conf_file(const char *file_name, struct acc100_conf *acc100_conf);

#endif /* _ACC100_CFG_APP_H_ */
