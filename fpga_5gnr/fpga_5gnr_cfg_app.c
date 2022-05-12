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
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include <errno.h>

#include "fpga_5gnr_cfg_app.h"
#include "cfg_reader.h"

extern int
fpga_5gnr_parse_conf_file(const char *file_name,
		struct fpga_5gnr_fec_conf *fpga_conf);

/* Read 8-bit register of FPGA 5GNR FEC device */
static uint8_t
fpga_reg_read_8(void *mmio_base, uint32_t offset)
{
	void *reg_addr = mmio_base + offset;
	return *((volatile uint8_t *)(reg_addr));
}

/* Read 16-bit register of FPGA 5GNR FEC device */
static uint16_t
fpga_reg_read_16(void *mmio_base, uint32_t offset)
{
	void *reg_addr = mmio_base + offset;
	uint16_t ret = *((volatile uint16_t *)(reg_addr));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	return __bswap_16(ret);
#else
	return ret;
#endif
}

/* Read 32-bit register of FPGA 5GNR FEC device */
static uint32_t
fpga_reg_read_32(void *mmio_base, uint32_t offset)
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
fpga_reg_write_16(void *mmio_base, uint32_t offset,
		uint16_t payload) {
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_16(payload);
#endif
	*((volatile uint16_t *) (reg_addr)) = payload;
}

static inline void
fpga_reg_write_32(void *mmio_base, uint32_t offset,
		uint32_t payload)
{
	void *reg_addr = mmio_base + offset;
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	payload = __bswap_32(payload);
#endif
	*((volatile uint32_t *) (reg_addr)) = payload;
}

static inline void
set_default_fpga_conf(struct fpga_5gnr_fec_conf *def_conf)
{
	/* Set pf mode to true */
	def_conf->pf_mode_en = true;

	/* Set ratio between UL and DL to 1:1 (unit of weight is 3 CBs) */
	def_conf->ul_bandwidth = 3;
	def_conf->dl_bandwidth = 3;

	/* Set Load Balance Factor to 64 */
	def_conf->dl_load_balance = 64;
	def_conf->ul_load_balance = 64;
}

static int
fpga_read_config_file(const char *arg_cfg_filename,
		struct fpga_5gnr_fec_conf *fpga_5gnr_fec_conf)
{
	bool unsafe_path = cfg_file_check_path_safety(arg_cfg_filename);
	if (unsafe_path == true) {
		printf("error, config file path \"%s\" is not safe",
				arg_cfg_filename);
		return -1;
	} else
		return fpga_5gnr_parse_conf_file(arg_cfg_filename,
				fpga_5gnr_fec_conf);
}

/* Read Static Register of FPGA 5GNR FEC device */
static inline void
print_static_reg_debug_info(void *mmio_base)
{
	uint8_t i, q_id;
	uint32_t fid;
	uint32_t version_id = fpga_reg_read_32(mmio_base,
			FPGA_5GNR_FEC_VERSION_ID);
	uint16_t config = fpga_reg_read_16(mmio_base,
			FPGA_5GNR_FEC_CONFIGURATION);
	uint8_t qmap_done = fpga_reg_read_8(mmio_base,
			FPGA_5GNR_FEC_QUEUE_PF_VF_MAP_DONE);
	uint16_t lb_factor = fpga_reg_read_16(mmio_base,
			FPGA_5GNR_FEC_LOAD_BALANCE_FACTOR);
	uint16_t ring_desc_len = fpga_reg_read_16(mmio_base,
			FPGA_5GNR_FEC_RING_DESC_LEN);

	printf("FEC FPGA RTL v%u.%u\n",
		((uint16_t)(version_id >> 16)), ((uint16_t)version_id));
	printf("UL.DL Weights = %u.%u\n",
			((uint8_t)config), ((uint8_t)(config >> 8)));
	printf("UL.DL Load Balance = %u.%u\n",
			((uint8_t)lb_factor), ((uint8_t)(lb_factor >> 8)));
	printf("Queue-PF/VF Mapping Table = %s\n",
			(qmap_done > 0) ? "READY" : "NOT-READY");
	printf("Ring Descriptor Size = %u bytes\n",
			ring_desc_len*FPGA_RING_DESC_LEN_UNIT_BYTES);

	printf("\n--------+-----+-----+-----+-----+-----+-----+-----+-----+-----+\n");
	printf("        |  PF | VF0 | VF1 | VF2 | VF3 | VF4 | VF5 | VF6 | VF7 |\n");
	printf("--------+-----+-----+-----+-----+-----+-----+-----+-----+-----+\n");

	for (q_id = 0; q_id < FPGA_TOTAL_NUM_QUEUES; q_id++) {

		printf("%s-Q'%02u |",
			(q_id < FPGA_NUM_UL_QUEUES) ? "UL" : "DL", q_id);

		fid = fpga_reg_read_32(mmio_base,
				FPGA_5GNR_FEC_QUEUE_MAP + (q_id << 2));

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
fpga_write_config(void *mapaddr, struct fpga_5gnr_fec_conf *conf)
{

	uint32_t payload_32, address;
	uint16_t payload_16;
	uint16_t q_id, vf_id, total_q_id, total_ul_q_id, total_dl_q_id;

	uint32_t *bar0addr = mapaddr;

	/*
	 * Configure UL:DL ratio.
	 * [7:0]: UL weight
	 * [15:8]: DL weight
	 */
	payload_16 = (conf->dl_bandwidth << 8) | conf->ul_bandwidth;
	address = FPGA_5GNR_FEC_CONFIGURATION;
	fpga_reg_write_16(bar0addr, address, payload_16);

	/* Clear all queues registers */
	payload_32 = FPGA_INVALID_HW_QUEUE_ID;
	for (q_id = 0; q_id < FPGA_TOTAL_NUM_QUEUES; ++q_id) {
		address = (q_id << 2) + FPGA_5GNR_FEC_QUEUE_MAP;
		fpga_reg_write_32(bar0addr, address, payload_32);
	}

	/*
	 * If PF mode is enabled allocate all queues for PF only.
	 *
	 * For VF mode each VF can have different number of UL and DL queues.
	 * Total number of queues to configure cannot exceed FPGA
	 * capabilities - 64 queues - 32 queues for UL and 32 queues for DL.
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
		for (q_id = 0; q_id < FPGA_TOTAL_NUM_QUEUES; ++q_id) {
			address = (q_id << 2) + FPGA_5GNR_FEC_QUEUE_MAP;
			fpga_reg_write_32(bar0addr, address, payload_32);
		}
	} else {
		/* Calculate total number of UL and DL queues to configure */
		total_ul_q_id = total_dl_q_id = 0;
		for (vf_id = 0; vf_id < FPGA_5GNR_FEC_NUM_VFS; ++vf_id) {
			total_ul_q_id += conf->vf_ul_queues_number[vf_id];
			total_dl_q_id += conf->vf_dl_queues_number[vf_id];
		}
		total_q_id = total_dl_q_id + total_ul_q_id;
		/*
		 * Check if total number of queues to configure does not exceed
		 * FPGA capabilities (64 queues - 32 UL and 32 DL queues)
		 */
		if ((total_ul_q_id > FPGA_NUM_UL_QUEUES) ||
			(total_dl_q_id > FPGA_NUM_DL_QUEUES) ||
			(total_q_id > FPGA_TOTAL_NUM_QUEUES)) {
			printf(
					"FPGA Configuration failed. Too many queues to configure: UL_Q %u, DL_Q %u, FPGA_Q %u",
					total_ul_q_id, total_dl_q_id,
					FPGA_TOTAL_NUM_QUEUES);
			return -EINVAL;
		}
		total_ul_q_id = 0;
		for (vf_id = 0; vf_id < FPGA_5GNR_FEC_NUM_VFS; ++vf_id) {
			for (q_id = 0; q_id < conf->vf_ul_queues_number[vf_id];
					++q_id, ++total_ul_q_id) {
				address = (total_ul_q_id << 2) +
						FPGA_5GNR_FEC_QUEUE_MAP;
				payload_32 = ((0x80 + vf_id) << 16) | 0x1;
				fpga_reg_write_32(bar0addr, address,
						payload_32);
			}
		}
		total_dl_q_id = 0;
		for (vf_id = 0; vf_id < FPGA_5GNR_FEC_NUM_VFS; ++vf_id) {
			for (q_id = 0; q_id < conf->vf_dl_queues_number[vf_id];
					++q_id, ++total_dl_q_id) {
				address = ((total_dl_q_id + FPGA_NUM_UL_QUEUES)
						<< 2) + FPGA_5GNR_FEC_QUEUE_MAP;
				payload_32 = ((0x80 + vf_id) << 16) | 0x1;
				fpga_reg_write_32(bar0addr, address,
						payload_32);
			}
		}
	}

	/* Setting Load Balance Factor */
	payload_16 = (conf->dl_load_balance << 8) | (conf->ul_load_balance);
	address = FPGA_5GNR_FEC_LOAD_BALANCE_FACTOR;
	fpga_reg_write_16(bar0addr, address, payload_16);

	/* Setting length of ring descriptor entry */
	payload_16 = FPGA_RING_DESC_ENTRY_LENGTH;
	address = FPGA_5GNR_FEC_RING_DESC_LEN;
	fpga_reg_write_16(bar0addr, address, payload_16);

	/* Queue PF/VF mapping table is ready */
	payload_16 = 0x1;
	address = FPGA_5GNR_FEC_QUEUE_PF_VF_MAP_DONE;
	fpga_reg_write_16(bar0addr, address, payload_16);

	print_static_reg_debug_info(bar0addr);
	printf("Mode of operation = %s-mode\n",
			conf->pf_mode_en ? "PF" : "VF");

	return 0;
}

int
fpga_5gnr_configure(void *bar0addr, const char *cfg_filename)
{
	struct fpga_5gnr_fec_conf fpga_conf;
	int ret;

	ret = fpga_read_config_file(cfg_filename, &fpga_conf);
	if (ret != 0) {
		printf("Error reading config file.\n");
		return -1;
	}

	ret = fpga_write_config(bar0addr, &fpga_conf);
	if (ret != 0) {
		printf("Error writing configuration for FPGA.\n");
		return -1;
	}

	return 0;
}
