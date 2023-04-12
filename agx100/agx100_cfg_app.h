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

#ifndef _AGX100_CFG_APP_H_
#define _AGX100_CFG_APP_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include <errno.h>

/* Multiplier of 256 bits (32 bytes) */
#define AGX100_RING_DESC_ENTRY_LENGTH (8)
#define AGX100_RING_DESC_LEN_UNIT_BYTES (64)
/* Maximum size of queue */
#define AGX100_RING_MAX_SIZE (1024)
#define AGX100_INVALID_HW_QUEUE_ID (0xFFFFFFFF)

/* AGX100 Register mapping on BAR0 */
enum {
	AGX100_VERSION_QUEUES_REG = 0x00000000, /**< len: 4B */
	AGX100_QUEUE_PF_VF_MAP_DONE = 0x00000008, /**< len: 1B */
	AGX100_LOAD_BALANCE_FACTOR = 0x0000000A, /**< len: 2B */
	AGX100_RING_DESC_LEN = 0x0000000C, /**< len: 2B */
	AGX100_FLR_TIME_OUT = 0x0000000E, /**< len: 2B */
	AGX100_VFQ_FLUSH_STATUS_LW = 0x00000018, /**< len: 4B */
	AGX100_VFQ_FLUSH_STATUS_HI = 0x0000001C, /**< len: 4B */
	AGX100_QUEUE_MAP = 0x00000100, /**< len: 256B */
	AGX100_RING_CTRL_REGS = 0x00000200, /**< len: 2048B */
	AGX100_DDR4_WR_ADDR_REGS = 0x00000A00, /**< len: 4B */
	AGX100_DDR4_WR_DATA_REGS = 0x00000A08, /**< len: 8B */
	AGX100_DDR4_WR_DONE_REGS = 0x00000A10, /**< len: 1B */
	AGX100_DDR4_RD_ADDR_REGS = 0x00000A18, /**< len: 4B */
	AGX100_DDR4_RD_DONE_REGS = 0x00000A20, /**< len: 1B */
	AGX100_DDR4_RD_RDY_REGS = 0x00000A28, /**< len: 1B */
	AGX100_DDR4_RD_DATA_REGS = 0x00000A30, /**< len: 8B */
	AGX100_DDR4_ADDR_RDY_REGS = 0x00000A38, /**< len: 1B */
	AGX100_HARQ_BUF_SIZE_RDY_REGS = 0x00000A40, /**< len: 1B */
	AGX100_HARQ_BUF_SIZE_REGS = 0x00000A48, /**< len: 4B */
	AGX100_MUTEX = 0x00000A60, /**< len: 4B */
	AGX100_MUTEX_RESET = 0x00000A68  /**< len: 4B */
};

/* AGX100 Ring Control Registers */
enum {
	AGX100_RING_HEAD_ADDR = 0x00000008,
	AGX100_RING_SIZE = 0x00000010,
	AGX100_RING_MISC = 0x00000014,
	AGX100_RING_ENABLE = 0x00000015,
	AGX100_RING_FLUSH_QUEUE_EN = 0x00000016,
	AGX100_RING_SHADOW_TAIL = 0x00000018,
	AGX100_RING_HEAD_POINT = 0x0000001C
};

/**< Number of Virtual Functions AGX100 supports */
#define AGX100_NUM_VFS 8

struct
agx100_conf {
	/**< 1 if PF is used for dataplane, 0 for VFs */
	bool pf_mode_en;
	/**< Number of UL queues per VF */
	uint8_t vf_ul_queues_number[AGX100_NUM_VFS];
	/**< Number of DL queues per VF */
	uint8_t vf_dl_queues_number[AGX100_NUM_VFS];
	/**< UL Load Balance */
	uint8_t ul_load_balance;
	/**< DL Load Balance */
	uint8_t dl_load_balance;
};

#endif /* _AGX100_CFG_APP_H_ */
