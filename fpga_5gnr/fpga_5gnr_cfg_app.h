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

#ifndef _FPGA_5GNR_CFG_APP_H_
#define _FPGA_5GNR_CFG_APP_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include <errno.h>

#define FPGA_5GNR_FEC_CONFIG_FILE_ENV "FPGA_5GNR_FEC_CONFIG_FILE"
#define FPGA_5GNR_FEC_CONFIG_FILE_NAME "fpga_5gnr_fec_config.cfg"

/* Multiplier of 256 bits (32 bytes) */
#define FPGA_RING_DESC_ENTRY_LENGTH (8)
#define FPGA_RING_DESC_LEN_UNIT_BYTES (32)
/* Maximum size of queue */
#define FPGA_RING_MAX_SIZE (1024)

#define FPGA_NUM_UL_QUEUES (32)
#define FPGA_NUM_DL_QUEUES (32)
#define FPGA_TOTAL_NUM_QUEUES (FPGA_NUM_UL_QUEUES + FPGA_NUM_DL_QUEUES)

#define FPGA_INVALID_HW_QUEUE_ID (0xFFFFFFFF)

/* FPGA 5GNR FEC Register mapping on BAR0 */
enum {
	FPGA_5GNR_FEC_VERSION_ID = 0x00000000, /* len: 4B */
	FPGA_5GNR_FEC_CONFIGURATION = 0x00000004, /* len: 2B */
	FPGA_5GNR_FEC_QUEUE_PF_VF_MAP_DONE = 0x00000008, /* len: 1B */
	FPGA_5GNR_FEC_LOAD_BALANCE_FACTOR = 0x0000000a, /* len: 2B */
	FPGA_5GNR_FEC_RING_DESC_LEN = 0x0000000c, /* len: 2B */
	FPGA_5GNR_FEC_VFQ_FLUSH_STATUS_LW = 0x00000018, /* len: 4B */
	FPGA_5GNR_FEC_VFQ_FLUSH_STATUS_HI = 0x0000001c, /* len: 4B */
	FPGA_5GNR_FEC_QUEUE_MAP = 0x00000040, /* len: 256B */
	FPGA_5GNR_FEC_RING_CTRL_REGS = 0x00000200, /* len: 2048B */
	FPGA_5GNR_FEC_DDR4_WR_ADDR_REGS = 0x00000A00, /* len: 4B */
	FPGA_5GNR_FEC_DDR4_WR_DATA_REGS = 0x00000A08, /* len: 8B */
	FPGA_5GNR_FEC_DDR4_WR_DONE_REGS = 0x00000A10, /* len: 1B */
	FPGA_5GNR_FEC_DDR4_RD_ADDR_REGS = 0x00000A18, /* len: 4B */
	FPGA_5GNR_FEC_DDR4_RD_DONE_REGS = 0x00000A20, /* len: 1B */
	FPGA_5GNR_FEC_DDR4_RD_RDY_REGS = 0x00000A28, /* len: 1B */
	FPGA_5GNR_FEC_DDR4_RD_DATA_REGS = 0x00000A30, /* len: 8B */
	FPGA_5GNR_FEC_DDR4_ADDR_RDY_REGS = 0x00000A38, /* len: 1B */
	FPGA_5GNR_FEC_HARQ_BUF_SIZE_RDY_REGS = 0x00000A40, /* len: 1B */
	FPGA_5GNR_FEC_HARQ_BUF_SIZE_REGS = 0x00000A48, /* len: 4B */
	FPGA_5GNR_FEC_MUTEX = 0x00000A60, /* len: 4B */
	FPGA_5GNR_FEC_MUTEX_RESET = 0x00000A68  /* len: 4B */
};

/* FPGA 5GNR FEC Ring Control Registers */
enum {
	FPGA_5GNR_FEC_RING_HEAD_ADDR = 0x00000008,
	FPGA_5GNR_FEC_RING_SIZE = 0x00000010,
	FPGA_5GNR_FEC_RING_MISC = 0x00000014,
	FPGA_5GNR_FEC_RING_ENABLE = 0x00000015,
	FPGA_5GNR_FEC_RING_FLUSH_QUEUE_EN = 0x00000016,
	FPGA_5GNR_FEC_RING_SHADOW_TAIL = 0x00000018,
	FPGA_5GNR_FEC_RING_HEAD_POINT = 0x0000001C
};

/**< Number of Virtual Functions FGPA 5GNR FEC supports */
#define FPGA_5GNR_FEC_NUM_VFS 8

struct
fpga_5gnr_fec_conf {
	/**< 1 if PF is used for dataplane, 0 for VFs */
	bool pf_mode_en;
	/**< Number of UL queues per VF */
	uint8_t vf_ul_queues_number[FPGA_5GNR_FEC_NUM_VFS];
	/**< Number of DL queues per VF */
	uint8_t vf_dl_queues_number[FPGA_5GNR_FEC_NUM_VFS];
	/**< UL bandwidth. Needed for schedule algorithm */
	uint8_t ul_bandwidth;
	/**< DL bandwidth. Needed for schedule algorithm */
	uint8_t dl_bandwidth;
	/**< UL Load Balance */
	uint8_t ul_load_balance;
	/**< DL Load Balance */
	uint8_t dl_load_balance;
};

/**
 * Configure FPGA
 */
int fpga_5gnr_configure(void *bar0addr, const char *arg_cfg_filename);

#endif /* _FPGA_5GNR_CFG_APP_H_ */
