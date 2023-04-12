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
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include <errno.h>

#include "agx100_cfg_app.h"
#include "cfg_reader.h"

extern int
agx100_parse_conf_file(const char *file_name,
		struct agx100_conf *agx100_conf);

/* Read 8-bit register of AGX100 device */
static uint8_t
agx100_reg_read_8(void *mmio_base, uint32_t offset)
{
	void *reg_addr = mmio_base + offset;
	return *((volatile uint8_t *)(reg_addr));
}

/* Read 16-bit register of AGX100 device */
static uint16_t
agx100_reg_read_16(void *mmio_base, uint32_t offset)
{
	void *reg_addr = mmio_base + offset;
	uint16_t ret = *((volatile uint16_t *)(reg_addr));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	return __bswap_16(ret);
#else
	return ret;
#endif
}

/* Read 32-bit register of AGX100 device */
static uint32_t
agx100_reg_read_32(void *mmio_base, uint32_t offset)
{
	void *reg_addr = mmio_base + offset;
	uint32_t ret = *((volatile uint32_t *)(reg_addr));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	return __bswap_32(ret);
#else
	return ret;
#endif
}

static inline void
agx100_reg_write_16(void *mmio_base, uint32_t offset,
		uint16_t payload) {
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_16(payload);
#endif
	*((volatile uint16_t *) (reg_addr)) = payload;
}

static inline void
agx100_reg_write_32(void *mmio_base, uint32_t offset,
		uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
}

static inline void
set_default_agx100_conf(struct agx100_conf *def_conf)
{
	/* Set pf mode to true */
	def_conf->pf_mode_en = true;

	/* Set Load Balance Factor to 64 */
	def_conf->dl_load_balance = 64;
	def_conf->ul_load_balance = 64;
}

static int
agx100_read_config_file(const char *arg_cfg_filename,
		struct agx100_conf *agx100_conf)
{
	bool unsafe_path = cfg_file_check_path_safety(arg_cfg_filename);
	if (unsafe_path == true) {
		printf("error, config file path \"%s\" is not safe",
				arg_cfg_filename);
		return -1;
	} else
		return agx100_parse_conf_file(arg_cfg_filename,
				agx100_conf);
}

/* Read Static Register of AGX100 device */
static inline void
print_static_reg_debug_info(void *mmio_base)
{
	uint8_t i, q_id;
	uint32_t fid;
	uint32_t version_num_queues = agx100_reg_read_32(mmio_base, AGX100_VERSION_QUEUES_REG);
	uint8_t major_version_id = version_num_queues >> 16;
	uint8_t minor_version_id = version_num_queues >> 8;
	uint8_t patch_id = version_num_queues;
	uint8_t qmap_done = agx100_reg_read_8(mmio_base, AGX100_QUEUE_PF_VF_MAP_DONE);
	uint16_t lb_factor = agx100_reg_read_16(mmio_base, AGX100_LOAD_BALANCE_FACTOR);
	uint16_t ring_desc_len = agx100_reg_read_16(mmio_base, AGX100_RING_DESC_LEN);

	/* Maximum number of queues on device */
	uint8_t total_num_queues = version_num_queues >> 24;
	uint8_t num_ul_queues = total_num_queues >> 1;

	printf("AGX100 RTL v%u.%u.%u\n", major_version_id, minor_version_id, patch_id);
	printf("Max number of Queues = %u\n", total_num_queues);
	printf("UL.DL Load Balance = %u.%u\n",
			((uint8_t)lb_factor), ((uint8_t)(lb_factor >> 8)));
	printf("Queue-PF/VF Mapping Table = %s\n",
			(qmap_done > 0) ? "READY" : "NOT-READY");
	printf("Ring Descriptor Size = %u bytes\n",
			ring_desc_len*AGX100_RING_DESC_LEN_UNIT_BYTES);

	printf("\n--------+-----+-----+-----+-----+-----+-----+-----+-----+-----+\n");
	printf("        |  PF | VF0 | VF1 | VF2 | VF3 | VF4 | VF5 | VF6 | VF7 |\n");
	printf("--------+-----+-----+-----+-----+-----+-----+-----+-----+-----+\n");

	for (q_id = 0; q_id < total_num_queues; q_id++) {

		printf("%s-Q'%02u |",
			(q_id < num_ul_queues) ? "UL" : "DL", q_id);

		fid = agx100_reg_read_32(mmio_base,
				AGX100_QUEUE_MAP + (q_id << 2));

		for (i = 0; i < 9; ++i) {

			if (!((fid >> 16) & (0x80)) && i == 0) {
				printf("  X  |");
				continue;
			}

			if (((((fid >> 16) & (0x7f)) + 1) == i) &&
					((fid >> 16) & (0x80)))
				printf("  X  |");
			else
				printf("     |");
		}
		printf("\n");
	}
	printf("--------+-----+-----+-----+-----+-----+-----+-----+-----+-----+\n\n");
}

static int
agx100_write_config(void *dev, void *mapaddr, struct agx100_conf *conf)
{

	uint32_t payload_32, address;
	uint16_t payload_16;
	uint16_t q_id, vf_id, total_q_id, total_ul_q_id, total_dl_q_id;

	uint32_t *bar0addr = mapaddr;
	/* Maximum number of queues on device */
	uint8_t total_num_queues = agx100_reg_read_32(bar0addr, AGX100_VERSION_QUEUES_REG) >> 24;
	uint8_t num_ul_queues = total_num_queues >> 1;
	uint8_t num_dl_queues = total_num_queues >> 1;

	/* Clear all queues registers */
	payload_32 = AGX100_INVALID_HW_QUEUE_ID;
	for (q_id = 0; q_id < total_num_queues; ++q_id) {
		address = (q_id << 2) + AGX100_QUEUE_MAP;
		agx100_reg_write_32(bar0addr, address, payload_32);
	}

	/*
	 * If PF mode is enabled allocate all queues for PF only.
	 *
	 * For VF mode each VF can have different number of UL and DL queues.
	 * Total number of queues to configure cannot exceed AGX100 capabilities.
	 * Queues mapping is done according to configuration:
	 *
	 * UL queues:
	 * |                Q_ID              | VF_ID |
	 * |                 0                |   0   |
	 * |                ...               |   0   |
	 * | conf->vf_dl_queues_number[0] - 1 |   0   |
	 * | conf->vf_dl_queues_number[0]     |   1   |
	 * |                ...               |   1   |
	 * | conf->vf_dl_queues_number[1] - 1 |   1   |
	 * |                ...               |  ...  |
	 * | conf->vf_dl_queues_number[7] - 1 |   7   |
	 *
	 * DL queues:
	 * |                Q_ID              | VF_ID |
	 * |                 32               |   0   |
	 * |                ...               |   0   |
	 * | conf->vf_ul_queues_number[0] - 1 |   0   |
	 * | conf->vf_ul_queues_number[0]     |   1   |
	 * |                ...               |   1   |
	 * | conf->vf_ul_queues_number[1] - 1 |   1   |
	 * |                ...               |  ...  |
	 * | conf->vf_ul_queues_number[7] - 1 |   7   |
	 *
	 * Example of configuration:
	 * conf->vf_ul_queues_number[0] = 4;  -> 4 UL queues for VF0
	 * conf->vf_dl_queues_number[0] = 4;  -> 4 DL queues for VF0
	 * conf->vf_ul_queues_number[1] = 2;  -> 2 UL queues for VF1
	 * conf->vf_dl_queues_number[1] = 2;  -> 2 DL queues for VF1
	 *
	 * UL:
	 * | Q_ID | VF_ID |
	 * |   0  |   0   |
	 * |   1  |   0   |
	 * |   2  |   0   |
	 * |   3  |   0   |
	 * |   4  |   1   |
	 * |   5  |   1   |
	 *
	 * DL:
	 * | Q_ID | VF_ID |
	 * |  32  |   0   |
	 * |  33  |   0   |
	 * |  34  |   0   |
	 * |  35  |   0   |
	 * |  36  |   1   |
	 * |  37  |   1   |
	 */
	if (conf->pf_mode_en) {
		payload_32 = 0x1;
		for (q_id = 0; q_id < total_num_queues; ++q_id) {
			address = (q_id << 2) + AGX100_QUEUE_MAP;
			agx100_reg_write_32(bar0addr, address, payload_32);
		}
	} else {
		/* Calculate total number of UL and DL queues to configure */
		total_ul_q_id = total_dl_q_id = 0;
		for (vf_id = 0; vf_id < AGX100_NUM_VFS; ++vf_id) {
			total_ul_q_id += conf->vf_ul_queues_number[vf_id];
			total_dl_q_id += conf->vf_dl_queues_number[vf_id];
		}
		total_q_id = total_dl_q_id + total_ul_q_id;
		/*
		 * Check if total number of queues to configure does not exceed
		 * AGX100 capabilities
		 */
		if ((total_ul_q_id > num_ul_queues) ||
			(total_dl_q_id > num_dl_queues) ||
			(total_q_id > total_num_queues)) {
			printf(
					"AGX100 Configuration failed. Too many queues to configure: UL_Q %u, DL_Q %u, AGX100_Q %u",
					total_ul_q_id, total_dl_q_id,
					total_num_queues);
			return -EINVAL;
		}
		total_ul_q_id = 0;
		for (vf_id = 0; vf_id < AGX100_NUM_VFS; ++vf_id) {
			for (q_id = 0; q_id < conf->vf_ul_queues_number[vf_id];
					++q_id, ++total_ul_q_id) {
				address = (total_ul_q_id << 2) +
						AGX100_QUEUE_MAP;
				payload_32 = ((0x80 + vf_id) << 16) | 0x1;
				agx100_reg_write_32(bar0addr, address,
						payload_32);
			}
		}
		total_dl_q_id = 0;
		for (vf_id = 0; vf_id < AGX100_NUM_VFS; ++vf_id) {
			for (q_id = 0; q_id < conf->vf_dl_queues_number[vf_id];
					++q_id, ++total_dl_q_id) {
				address = ((total_dl_q_id + num_ul_queues)
						<< 2) + AGX100_QUEUE_MAP;
				payload_32 = ((0x80 + vf_id) << 16) | 0x1;
				agx100_reg_write_32(bar0addr, address,
						payload_32);
			}
		}
	}

	/* Setting Load Balance Factor */
	payload_16 = (conf->dl_load_balance << 8) | (conf->ul_load_balance);
	address = AGX100_LOAD_BALANCE_FACTOR;
	agx100_reg_write_16(bar0addr, address, payload_16);

	/* Setting length of ring descriptor entry */
	payload_16 = AGX100_RING_DESC_ENTRY_LENGTH;
	address = AGX100_RING_DESC_LEN;
	agx100_reg_write_16(bar0addr, address, payload_16);

	/* Queue PF/VF mapping table is ready */
	payload_16 = 0x1;
	address = AGX100_QUEUE_PF_VF_MAP_DONE;
	agx100_reg_write_16(bar0addr, address, payload_16);

	print_static_reg_debug_info(bar0addr);
	printf("Mode of operation = %s-mode\n",
			conf->pf_mode_en ? "PF" : "VF");

	return 0;
}

int
agx100_configure(void *dev, void *bar0addr, const char *cfg_filename, const bool first_cfg)
{
	struct agx100_conf agx100_conf;
	int ret;

	ret = agx100_read_config_file(cfg_filename, &agx100_conf);
	if (ret != 0) {
		printf("Error reading config file.\n");
		return -1;
	}

	ret = agx100_write_config(dev, bar0addr, &agx100_conf);
	if (ret != 0) {
		printf("Error writing configuration for AGX100.\n");
		return -1;
	}

	return 0;
}
