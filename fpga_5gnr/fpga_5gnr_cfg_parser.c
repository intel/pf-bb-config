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
#include <errno.h>

#include "cfg_reader.h"
#include "fpga_5gnr_cfg_app.h"

/* Names of sections used in the configuration file */
#define MODE "MODE"
#define UL "UL"
#define DL "DL"

/* Names of entries in sections used in the configuration file */
#define PFMODE "pf_mode_en"
#define BANDWIDTH "bandwidth"
#define LOAD_BALANCE "load_balance"
#define QUEUE_MAP "vfqmap"

/* Default values for FPGA device configuration variables */
#define DEFAULT_PF_MODE_EN 1
#define DEFAULT_UL_BANDWIDTH 3
#define DEFAULT_DL_BANDWIDTH 3
#define DEFAULT_UL_LOAD_BALANCE 64
#define DEFAULT_DL_LOAD_BALANCE 64
#define DEFAULT_NUM_VF_QUEUE 8

/* Possible values for MODE and LLR_SIGN */
#define ZERO "0"
#define ONE "1"

static int
parse_number8(const char *str, uint8_t *value)
{
	uint64_t val = 0;
	char *end;

	if (str == NULL)
		return -EINVAL;

	val = strtoul(str, &end, 0);
	if (val > UINT8_MAX || str == end) {
		printf("ERROR: Invalid value %" PRIu64 "\n", val);
		return -ERANGE;
	}

	*value = (uint8_t) val;
	return 1;
}

static int
parse_array8(const char *str, uint8_t *array)
{
	int i;
	uint64_t val = 0;
	char *end;

	if (str == NULL)
		return -EINVAL;

	char *val_ch = strtok((char *)str, ",");
	for (i = 0; i < 8 && NULL != val_ch; i++) {

		val = strtoul(val_ch, &end, 0);
		if (val > UINT8_MAX || val_ch == end) {
			printf("ERROR: Invalid value %" PRIu64 "\n", val);
			return -ERANGE;
		}
		array[i] = (uint8_t) val;

		val_ch = strtok(NULL, ",");
	}
	return 1;
}

static void
set_default_config(struct fpga_5gnr_fec_conf *fpga_conf)
{
	int i;

	/* Set pf mode to true */
	fpga_conf->pf_mode_en = DEFAULT_PF_MODE_EN;

	/* Set ratio between UL and DL to 1:1 (unit of weight is 3 CBs) */
	fpga_conf->ul_bandwidth = DEFAULT_UL_BANDWIDTH;
	fpga_conf->dl_bandwidth = DEFAULT_DL_BANDWIDTH;

	/* Set Load Balance Factor to 64 */
	fpga_conf->ul_load_balance = DEFAULT_UL_LOAD_BALANCE;
	fpga_conf->dl_load_balance = DEFAULT_DL_LOAD_BALANCE;

	for (i = 0; i < FPGA_5GNR_FEC_NUM_VFS; i++) {
		fpga_conf->vf_ul_queues_number[i] = DEFAULT_NUM_VF_QUEUE / 2;
		fpga_conf->vf_dl_queues_number[i] = DEFAULT_NUM_VF_QUEUE / 2;
	}
}

static int
fpga_handler(void *user, const char *section,
	     const char *name, const char *value)
{
	struct fpga_5gnr_fec_conf *fpga_conf =
			(struct fpga_5gnr_fec_conf *) user;
	int ret = 1;

	if (!strcmp(section, MODE) && !strcmp(name, PFMODE)) {
		if (!strcmp(value, ZERO))
			fpga_conf->pf_mode_en = false;
		else if (!strcmp(value, ONE))
			fpga_conf->pf_mode_en = true;
		else
			ret = 0;
	} else if (!strcmp(section, UL) && !strcmp(name, BANDWIDTH)) {
		ret = parse_number8(value, &fpga_conf->ul_bandwidth);
	} else if (!strcmp(section, DL) && !strcmp(name, BANDWIDTH)) {
		ret = parse_number8(value, &fpga_conf->dl_bandwidth);
	} else if (!strcmp(section, UL) && !strcmp(name, LOAD_BALANCE)) {
		ret = parse_number8(value, &fpga_conf->ul_load_balance);
	} else if (!strcmp(section, DL) && !strcmp(name, LOAD_BALANCE)) {
		ret = parse_number8(value, &fpga_conf->dl_load_balance);
	} else if (!strcmp(section, UL) && !strcmp(name, QUEUE_MAP)) {
		ret = parse_array8(value, fpga_conf->vf_ul_queues_number);
	} else if (!strcmp(section, DL) && !strcmp(name, QUEUE_MAP)) {
		ret = parse_array8(value, fpga_conf->vf_dl_queues_number);
	} else {
		printf("ERROR: Section (%s) or name (%s) is not valid.\n",
		       section, name);
		return 0;
	}
	if (ret != 1)
		printf("Error: Conversion of value (%s) failed.\n", value);

	return ret;
}

int
fpga_5gnr_parse_conf_file(const char *file_name,
		struct fpga_5gnr_fec_conf *fpga_conf)
{
	int ret;

	set_default_config(fpga_conf);

	ret = cfg_parse(file_name, fpga_handler, fpga_conf);

	if (ret == -1) {
		printf("ERROR: Error loading configuration file %s\n",
			file_name);
		set_default_config(fpga_conf);
		return -ENOENT;
	} else if (ret == -2) {
		printf("ERROR: Memory allocation error\n");
		set_default_config(fpga_conf);
		return -ENOMEM;
	}

	return 0;
}
