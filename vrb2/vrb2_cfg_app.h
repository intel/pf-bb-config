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

#ifndef _VRB2_CFG_APP_H_
#define _VRB2_CFG_APP_H_

#include "acc100_cfg_app.h"

#define VRB2_BYTES_IN_WORD 4

#define VRB2_NUM_VFS            64
#define VRB2_NUM_QGRPS          32
#define VRB2_NUM_QGRPS_PER_WORD 8
#define VRB2_NUM_AQS            64
#define VRB2_MAX_QDEPTH         8
#define MAX_ENQ_BATCH_SIZE        255
#define VRB2_NUM_TMPL           32
#define VRB2_AQ_REG_NUM         4
#define VF_OFFSET_QOS 16 /* offset in Memory Space specific to QoS Mon */
#define VRB2_MON_OFFSET      0x10000
#define VRB2_MON_NUMS        6
#define VRB2_TMPL_PRI_0      0x03020100
#define VRB2_TMPL_PRI_1      0x07060504
#define VRB2_TMPL_PRI_2      0x0b0a0908
#define VRB2_TMPL_PRI_3      0x0f0e0d0c
#define VRB2_TMPL_PRI_4      0x13121110
#define VRB2_TMPL_PRI_5      0x17161514
#define VRB2_TMPL_PRI_6      0x1b1a1918
#define VRB2_TMPL_PRI_7      0x1f1e1d1c
#define VRB2_WORDS_IN_ARAM_SIZE (512 * 1024 / 4)

#define VRB2_SIG_UL_5G       0
#define VRB2_SIG_UL_5G_LAST  5
#define VRB2_SIG_DL_5G       9
#define VRB2_SIG_DL_5G_LAST 11
#define VRB2_SIG_UL_4G      12
#define VRB2_SIG_UL_4G_LAST 16
#define VRB2_SIG_DL_4G      21
#define VRB2_SIG_DL_4G_LAST 23
#define VRB2_SIG_FFT        24
#define VRB2_SIG_FFT_LAST   26
#define VRB2_SIG_MLD        30
#define VRB2_SIG_MLD_LAST   31
#define VRB2_FFT_NUM        3

#define VRB2_IDMAP_UL_5G       0
#define VRB2_IDMAP_UL_5G_LAST  5
#define VRB2_IDMAP_DL_5G       6
#define VRB2_IDMAP_DL_5G_LAST  8
#define VRB2_IDMAP_UL_4G       9
#define VRB2_IDMAP_UL_4G_LAST 13
#define VRB2_IDMAP_DL_4G      14
#define VRB2_IDMAP_DL_4G_LAST 16
#define VRB2_IDMAP_FFT        17
#define VRB2_IDMAP_FFT_LAST   19
#define VRB2_IDMAP_MLD        20
#define VRB2_IDMAP_MLD_LAST   21

#define VRB2_NUM_ACCS       6
#define VRB2_ACCMAP_0       0
#define VRB2_ACCMAP_1       2
#define VRB2_ACCMAP_2       1
#define VRB2_ACCMAP_3       3
#define VRB2_ACCMAP_4       4
#define VRB2_ACCMAP_5       5
#define VRB2_PF_VAL         2

/* VRB2 Configuration */
#define VRB2_FABRIC_MODE      0x8000103
#define VRB2_CFG_DMA_ERROR    0x7DF
#define VRB2_CFG_AXI_CACHE    0x22
#define VRB2_CFG_QMGR_HI_P    0x0F0F
#define VRB2_CLOCK_GATING_EN  0x30000
#define VRB2_ENGINE_OFFSET    0x1000
#define VRB2_MS_IN_US         (1000)
#define VRB2_QMGR_ECC         0x3
#define VRB2_FABRIC_OFFSET    0x10

#define VRB2_FFT_CFG_0        0x2001
/* Cannot enable global clock gating on A0. */
#define VRB2_CLK_EN           0x00030A00
#define VRB2_CLK_DIS          0x03F30A00
#define VRB2_FFT_ECC          0x60
#define VRB2_FFT_RAM_EN       0x80000000
#define VRB2_FFT_RAM_DIS      0x0
#define VRB2_FFT_RAM_SIZE     512
#define VRB2_FFT_PAGE_SIZE    512
#define VRB2_FFT_FIST_OFFSET  256
#define VRB2_PG_MASK_0        0x1F
#define VRB2_PG_MASK_1        0xF
#define VRB2_PG_MASK_2        0x1
#define VRB2_PG_MASK_3        0x0
#define VRB2_PG_MASK_FFT      1
#define VRB2_PG_MASK_4GUL     4
#define VRB2_PG_MASK_5GUL     8
#define VRB2_QSTRIDE_MAP      0x1F07F0
#define VRB2_PROC_TIMEOUT     (0x2000   * 0x100) /* 1ms FIXME B0 x 0x100. */
#define VRB2_QMGR_TIMEOUT     (0x100000 * 0x100) /* 1ms FIXME B0 x 0x100. */
#define VRB2_CLUST_TIMEOUT    0x100000 /* 1ms */
#define VRB2_QMGR_ARAM_TIMEOUT 0x800 /* 2k cycles */
#define VRB2_QMGR_AXI_TIMEOUT 0x800 /* 2k cycles */
#define VRB2_ARAM_CONTROL     0x30
#define VRB2_ENG_TYPE_NUM     6
#define VRB2_ERR_TYPE_PROC_TO 8
#define VRB2_ERR_TYPE_OFFSET  10
#define VRB2_HIERR_MASK       0xFFFFFFEF /* Remove proc time out */
#define VRB2_EXTERR_MASK      0xFFFDFFFF /* Remove proc time out */
#define VRB2_CHR_ENABLE       0x3
#define VRB2_CHR_THOLD        0x186A0
#define VRB2_DMA_ERROR_OR_CHR 18
#define VRB2_NUM_QOS           6
#define VRB2_BLOCK_ON_TX       0xC7478807
#define VRB2_A0_VALUE          0x7
#define VRB2_B0_VALUE          0xF
#define VRB2_ARB_QDEPTH        2
#define VRB2_UNUSED              9
#define VRB2_UL_5G_ROUTER_SIZE   3 /* 2 KB */
#define VRB2_DEFAULT_ROUTER_SIZE 2 /* 1 KB */
#define VRB2_DMA_WEIGHT          0x1111
#define VRB2_IB_THHOLD           0x20200208
#define VRB2_STREAM_IB_THHOLD    0x00001008
#define VRB2_DMA_SWITCH_DEFAULT  20
#define VRB2_DMA_SWITCH_STREAM   4
#define VRB2_N0_DEPTH_LOG2       5

#define VRB2_LUT_SIZE (16*(16+32+64+128+256+512+1024))

#define VRB2_INFO_RING_NUM_ENTRIES 1024

/* Mask used to calculate an index in an Info Ring array (not a byte offset) */
#define VRB2_INFO_RING_MASK        (VRB2_INFO_RING_NUM_ENTRIES - 1)
#define VRB2_INFO_RING_PTR_MASK    ((VRB2_INFO_RING_NUM_ENTRIES * 4) - 1)

#define INFO_RING_DET_INFO		GENMASK(16, 0)
#define INFO_RING_DET_INFO_AQ_ID	GENMASK(5, 0)
#define INFO_RING_DET_INFO_QG_ID	GENMASK(10, 6)
#define INFO_RING_DET_INFO_VF_ID	GENMASK(16, 11)
#define INFO_RING_DET_INFO_FEC_SLICE	GENMASK(10, 0)
#define INFO_RING_DET_INFO_SLAVE_ID	GENMASK(1, 0)
#define INFO_RING_DET_INFO_MASTER_ID	GENMASK(3, 2)
#define INFO_RING_DET_INFO_WRITE	BIT(4)
#define INFO_RING_DET_INFO_TRANS_ID	GENMASK(9, 5)
#define INFO_RING_INT_NB		GENMASK(22, 17)
#define INFO_RING_MSI_0			BIT(23)
#define INFO_RING_VF2PF			GENMASK(29, 24)
#define INFO_RING_ERR_TYPE		GENMASK(29, 24)
#define INFO_RING_LOOP			BIT(30)
#define INFO_RING_VALID			BIT(31)


#define VRB2_5GUL_ENGS 6
#define VRB2_5GDL_ENGS 3
#define VRB2_4GUL_ENGS 5
#define VRB2_4GDL_ENGS 3
#define VRB2_FFT_ENGS 3
#define VRB2_MLD_ENGS 1
#define VRB2_PMON_OFF_1 256
#define VRB2_PMON_OFF_2 16

#define VRB2_BUSMON_START 2
#define VRB2_BUSMON_RESET 1
#define VRB2_BUSMON_STOP  0
#define VRB2_CORE_DMA_ERROR        0x3
#define VRB2_CORE_RECOVERY_FAILURE 0x4800

/**
 * Structure to pass VRB2 configuration.
 * Note: all VF Bundles will have the same configuration.
 */
struct vrb2_conf {
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
	struct q_topology_t q_fft;  /**< FFT queues */
	struct q_topology_t q_mld;  /**< MLD queues */
	/** Uplink arbitration configuration */
	struct arbitration_t arb_ul_4g[VRB2_NUM_VFS];
	/** Downlink arbitration configuration */
	struct arbitration_t arb_dl_4g[VRB2_NUM_VFS];
	/** Uplink arbitration configuration */
	struct arbitration_t arb_ul_5g[VRB2_NUM_VFS];
	/** Downlink arbitration configuration */
	struct arbitration_t arb_dl_5g[VRB2_NUM_VFS];
	/** FFT arbitration configuration */
	struct arbitration_t arb_fft[VRB2_NUM_VFS];
	/** MLD arbitration configuration */
	struct arbitration_t arb_mld[VRB2_NUM_VFS];
};

typedef struct {
	char *name;
	bool use_det_info;
	bool fatal;
} vrb2_ir_int_type_info;

#define VRB2_INFO_RING_SIZE (VRB2_INFO_RING_NUM_ENTRIES * \
		sizeof(uint32_t))

/*
 * Configure VRB2
 */

extern int
vrb2_parse_conf_file(const char *file_name, struct vrb2_conf *vrb2_conf);

#endif /* _VRB2_CFG_APP_H_ */
