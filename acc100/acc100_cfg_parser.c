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

#include <ini.h>
#include "acc100_cfg_app.h"

/* Names of sections used in the configuration file */
#define MODE "MODE"
#define LLR_SIGN "LLR_SIGN"
#define VFBUNDLES "VFBUNDLES"
#define MAXQSIZE "MAXQSIZE"
#define QUL4G "QUL4G"
#define QDL4G "QDL4G"
#define QUL5G "QUL5G"
#define QDL5G "QDL5G"
#define ARBUL4G "ARBUL4G"
#define ARBDL4G "ARBDL4G"
#define ARBUL5G "ARBUL5G"
#define ARBDL5G "ARBDL5G"

/* Names of entries in sections used in the configuration file */
#define PFMODE "pf_mode_en"
#define INPUT_POS_LLR_1_BIT "input_pos_llr_1_bit"
#define OUTPUT_POS_LLR_1_BIT "output_pos_llr_1_bit"
#define NUMVF_BUNDLES "num_vf_bundles"
#define MAX_QUEUE_SIZE "max_queue_size"
#define NUM_QGROUPS "num_qgroups"
#define NUM_AQS_PER_GROUPS "num_aqs_per_groups"
#define AQ_DEPTH_LOG2 "aq_depth_log2"
#define ROUND_ROBIN_WEIGHT "round_robin_weight"
#define GBR_THRSH1 "gbr_threshold1"
#define GBR_THRSH2 "gbr_threshold2"

/* Default values for ACC100 device configuration variables */
#define NUM_OF_BUNDLES 1
#define DEFAULT_PF_MODE_EN 1
#define QDL_NUM_QGROUPS 1
#define QDL_NUM_AQS_PER_GROUPS 1
#define QDL_AQ_DEPTH_LOG2 8
#define QUL_NUM_QGROUPS 1
#define QUL_NUM_AQS_PER_GROUPS 1
#define QUL_AQ_DEPTH_LOG2 8
#define ARBDL_GBR_THRSH1 0
#define ARBDL_GBR_THRSH2 0
#define ARBDL_ROUND_ROBIN_WEIGHT 1
#define ARBUL_GBR_THRSH1 0
#define ARBUL_GBR_THRSH2 0
#define ARBUL_ROUND_ROBIN_WEIGHT 1

/* Possible values for MODE and LLR_SIGN */
#define ZERO "0"
#define ONE "1"

/* Convert a character to a 16-bit integer */
static int
parse_number16(const char *str, uint16_t *value)
{
	uint64_t val = 0;
	char *end;

	if (str == NULL)
		return -EINVAL;

	val = strtoul(str, &end, 0);
	if (val > UINT16_MAX || str == end) {
		printf("ERROR: Invalid value %" PRIu64 "\n", val);
		return -ERANGE;
	}

	*value = (uint16_t) val;
	return 1;
}

/* Convert a character to a 32-bit integer */
static int
parse_number32(const char *str, uint32_t *value)
{
	uint64_t val = 0;
	char *end;

	if (str == NULL)
		return -EINVAL;

	val = strtoul(str, &end, 0);
	if (val > UINT32_MAX || str == end) {
		printf("ERROR: Invalid value %" PRIu64 "\n", val);
		return -ERANGE;
	}
	*value = (uint32_t) val;
	return 1;
}

/* Find the item number for a given item */
static int
parse_item_no(const char *full_name, const char *base_name)
{
	int postfix_len, number_start;
	uint16_t item_no = 0;

	postfix_len = strlen(full_name) - strlen(base_name);
	if (postfix_len > 0) {
		number_start = strlen((const char *) base_name);
		parse_number16(&full_name[number_start], &item_no);
	}

	return item_no;
}

/* Set the default config for the device (if no other config file is given) */
static void
set_default_config(struct acc100_conf *acc100_conf)
{
	int x;

	acc100_conf->num_vf_bundles = NUM_OF_BUNDLES;
	acc100_conf->pf_mode_en = DEFAULT_PF_MODE_EN;

	acc100_conf->q_dl_4g.num_qgroups = QDL_NUM_QGROUPS;
	acc100_conf->q_dl_4g.num_aqs_per_groups = QDL_NUM_AQS_PER_GROUPS;
	acc100_conf->q_dl_4g.aq_depth_log2 = QDL_AQ_DEPTH_LOG2;
	acc100_conf->q_ul_4g.num_qgroups = QUL_NUM_QGROUPS;
	acc100_conf->q_ul_4g.num_aqs_per_groups = QUL_NUM_AQS_PER_GROUPS;
	acc100_conf->q_ul_4g.aq_depth_log2 = QUL_AQ_DEPTH_LOG2;
	acc100_conf->q_dl_5g.num_qgroups = QDL_NUM_QGROUPS;
	acc100_conf->q_dl_5g.num_aqs_per_groups = QDL_NUM_AQS_PER_GROUPS;
	acc100_conf->q_dl_5g.aq_depth_log2 = QDL_AQ_DEPTH_LOG2;
	acc100_conf->q_ul_5g.num_qgroups = QUL_NUM_QGROUPS;
	acc100_conf->q_ul_5g.num_aqs_per_groups = QUL_NUM_AQS_PER_GROUPS;
	acc100_conf->q_ul_5g.aq_depth_log2 = QUL_AQ_DEPTH_LOG2;

	for (x = 0; x < ACC100_NUM_VFS; x++) {
		acc100_conf->arb_dl_4g[x].gbr_threshold1 = ARBDL_GBR_THRSH1;
		acc100_conf->arb_dl_4g[x].gbr_threshold2 = ARBDL_GBR_THRSH2;
		acc100_conf->arb_dl_4g[x].round_robin_weight =
				ARBDL_ROUND_ROBIN_WEIGHT;
	}
	for (x = 0; x < ACC100_NUM_VFS; x++) {
		acc100_conf->arb_ul_4g[x].gbr_threshold1 = ARBUL_GBR_THRSH1;
		acc100_conf->arb_ul_4g[x].gbr_threshold2 = ARBUL_GBR_THRSH2;
		acc100_conf->arb_ul_4g[x].round_robin_weight =
				ARBUL_ROUND_ROBIN_WEIGHT;
	}
	for (x = 0; x < ACC100_NUM_VFS; x++) {
		acc100_conf->arb_dl_5g[x].gbr_threshold1 = ARBDL_GBR_THRSH1;
		acc100_conf->arb_dl_5g[x].gbr_threshold2 = ARBDL_GBR_THRSH2;
		acc100_conf->arb_dl_5g[x].round_robin_weight =
				ARBDL_ROUND_ROBIN_WEIGHT;
	}
	for (x = 0; x < ACC100_NUM_VFS; x++) {
		acc100_conf->arb_ul_5g[x].gbr_threshold1 = ARBUL_GBR_THRSH1;
		acc100_conf->arb_ul_5g[x].gbr_threshold2 = ARBUL_GBR_THRSH2;
		acc100_conf->arb_ul_5g[x].round_robin_weight =
				ARBUL_ROUND_ROBIN_WEIGHT;
	}
}

/* Handler function for .ini parser (returns 1 for success) */
static int
acc100_handler(void *user, const char *section,
	    const char *name, const char *value)
{
	struct acc100_conf *acc100_conf = (struct acc100_conf *) user;
	int ret = 1, index;

	if (!strcmp(section, MODE) && !strcmp(name, PFMODE)) {
		if (!strcmp(value, ZERO))
			acc100_conf->pf_mode_en = false;
		else if (!strcmp(value, ONE))
			acc100_conf->pf_mode_en = true;
		else
			ret = 0;
	} else if (!strcmp(section, LLR_SIGN) &&
			!strcmp(name, INPUT_POS_LLR_1_BIT)) {
		if (!strcmp(value, ZERO))
			acc100_conf->input_pos_llr_1_bit = false;
		else if (!strcmp(value, ONE))
			acc100_conf->input_pos_llr_1_bit = true;
		else
			ret = 0;
	} else if (!strcmp(section, LLR_SIGN) &&
			!strcmp(name, OUTPUT_POS_LLR_1_BIT)) {
		if (!strcmp(value, ZERO))
			acc100_conf->output_pos_llr_1_bit = false;
		else if (!strcmp(value, ONE))
			acc100_conf->output_pos_llr_1_bit = true;
		else
			ret = 0;
	} else if (!strcmp(section, VFBUNDLES) &&
			!strcmp(name, NUMVF_BUNDLES)) {
		ret = parse_number16(value, &acc100_conf->num_vf_bundles);
	} else if (!strcmp(section, MAXQSIZE) &&
			!strcmp(name, MAX_QUEUE_SIZE)) {
		/* MAX_QUEUE_SIZE is not used. Keep it for parsing purposes */
	} else if (!strcmp(section, QUL4G) && !strcmp(name, NUM_QGROUPS)) {
		ret = parse_number16(value, &acc100_conf->q_ul_4g.num_qgroups);
	} else if (!strcmp(section, QUL4G) &&
			!strcmp(name, NUM_AQS_PER_GROUPS)) {
		ret = parse_number16(value,
				&acc100_conf->q_ul_4g.num_aqs_per_groups);
	} else if (!strcmp(section, QUL4G) && !strcmp(name, AQ_DEPTH_LOG2)) {
		ret = parse_number16(value,
				&acc100_conf->q_ul_4g.aq_depth_log2);
	} else if (!strcmp(section, QDL4G) && !strcmp(name, NUM_QGROUPS)) {
		ret = parse_number16(value, &acc100_conf->q_dl_4g.num_qgroups);
	} else if (!strcmp(section, QDL4G) &&
			!strcmp(name, NUM_AQS_PER_GROUPS)) {
		ret = parse_number16(value,
				&acc100_conf->q_dl_4g.num_aqs_per_groups);
	} else if (!strcmp(section, QDL4G) && !strcmp(name, AQ_DEPTH_LOG2)) {
		ret = parse_number16(value,
				&acc100_conf->q_dl_4g.aq_depth_log2);
	} else if (!strcmp(section, QUL5G) && !strcmp(name, NUM_QGROUPS)) {
		ret = parse_number16(value, &acc100_conf->q_ul_5g.num_qgroups);
	} else if (!strcmp(section, QUL5G) &&
			!strcmp(name, NUM_AQS_PER_GROUPS)) {
		ret = parse_number16(value,
				&acc100_conf->q_ul_5g.num_aqs_per_groups);
	} else if (!strcmp(section, QUL5G) && !strcmp(name, AQ_DEPTH_LOG2)) {
		ret = parse_number16(value,
				&acc100_conf->q_ul_5g.aq_depth_log2);
	} else if (!strcmp(section, QDL5G) && !strcmp(name, NUM_QGROUPS)) {
		ret = parse_number16(value, &acc100_conf->q_dl_5g.num_qgroups);
	} else if (!strcmp(section, QDL5G) &&
			!strcmp(name, NUM_AQS_PER_GROUPS)) {
		ret = parse_number16(value,
				&acc100_conf->q_dl_5g.num_aqs_per_groups);
	} else if (!strcmp(section, QDL5G) && !strcmp(name, AQ_DEPTH_LOG2)) {
		ret = parse_number16(value,
				&acc100_conf->q_dl_5g.aq_depth_log2);
	} else if (strstr(section, ARBUL4G) &&
			!strcmp(name, ROUND_ROBIN_WEIGHT)) {
		index = parse_item_no(section, ARBUL4G);
		ret = parse_number16(value,
			&acc100_conf->arb_ul_4g[index].round_robin_weight);
	} else if (strstr(section, ARBUL4G) && !strcmp(name, GBR_THRSH1)) {
		index = parse_item_no(section, ARBUL4G);
		ret = parse_number32(value,
				&acc100_conf->arb_ul_4g[index].gbr_threshold1);
	} else if (strstr(section, ARBUL4G) && !strcmp(name, GBR_THRSH2)) {
		index = parse_item_no(section, ARBUL4G);
		ret = parse_number32(value,
				&acc100_conf->arb_ul_4g[index].gbr_threshold2);
	} else if (strstr(section, ARBDL4G) &&
			!strcmp(name, ROUND_ROBIN_WEIGHT)) {
		index = parse_item_no(section, ARBDL4G);
		ret = parse_number16(value,
			&acc100_conf->arb_dl_4g[index].round_robin_weight);
	} else if (strstr(section, ARBDL4G) && !strcmp(name, GBR_THRSH1)) {
		index = parse_item_no(section, ARBDL4G);
		ret = parse_number32(value,
				&acc100_conf->arb_dl_4g[index].gbr_threshold1);
	} else if (strstr(section, ARBDL4G) && !strcmp(name, GBR_THRSH2)) {
		index = parse_item_no(section, ARBDL4G);
		ret = parse_number32(value,
				&acc100_conf->arb_dl_4g[index].gbr_threshold2);
	} else if (strstr(section, ARBUL5G) &&
			!strcmp(name, ROUND_ROBIN_WEIGHT)) {
		index = parse_item_no(section, ARBUL5G);
		ret = parse_number16(value,
			&acc100_conf->arb_ul_5g[index].round_robin_weight);
	} else if (strstr(section, ARBUL5G) && !strcmp(name, GBR_THRSH1)) {
		index = parse_item_no(section, ARBUL5G);
		ret = parse_number32(value,
				&acc100_conf->arb_ul_5g[index].gbr_threshold1);
	} else if (strstr(section, ARBUL5G) && !strcmp(name, GBR_THRSH2)) {
		index = parse_item_no(section, ARBUL5G);
		ret = parse_number32(value,
				&acc100_conf->arb_ul_5g[index].gbr_threshold2);
	} else if (strstr(section, ARBDL5G) &&
			!strcmp(name, ROUND_ROBIN_WEIGHT)) {
		index = parse_item_no(section, ARBDL5G);
		ret = parse_number16(value,
			&acc100_conf->arb_dl_5g[index].round_robin_weight);
	} else if (strstr(section, ARBDL5G) && !strcmp(name, GBR_THRSH1)) {
		index = parse_item_no(section, ARBDL5G);
		ret = parse_number32(value,
				&acc100_conf->arb_dl_5g[index].gbr_threshold1);
	} else if (strstr(section, ARBDL5G) && !strcmp(name, GBR_THRSH2)) {
		index = parse_item_no(section, ARBDL5G);
		ret = parse_number32(value,
				&acc100_conf->arb_dl_5g[index].gbr_threshold2);
	} else {
		printf("ERROR: section (%s) or name (%s) is not valid\n",
			section, name);
		ret = 0;
	}

	if (ret != 1)
		printf("ERROR: Conversion of value (%s) failed\n", value);

	return ret;
}

/* Enforce range check for a given queue configuration */
int
acc100_queue_range_check(struct q_topology_t q_conf)
{
	if ((q_conf.num_aqs_per_groups < 1) ||
			(q_conf.num_aqs_per_groups > ACC100_NUM_AQS)) {
		printf("ERROR: Number of AQs out of range %d\n",
				q_conf.num_aqs_per_groups);
		return -1;
	}
	if ((q_conf.aq_depth_log2 < 1) ||
			(q_conf.aq_depth_log2 > ACC100_MAX_QDEPTH)) {
		printf("ERROR: AQ Depth out of range %d\n",
				q_conf.aq_depth_log2);
		return -1;
	}
	if (q_conf.num_qgroups > ACC100_NUM_QGRPS) {
		printf("ERROR: Number of QG out of range %d\n",
				q_conf.num_qgroups);
		return -1;
	}
	return 0;
}

/* Enforce range check for a given device configuration */
int
acc100_range_check(struct acc100_conf *acc100_conf)
{
	uint16_t totalQgs = acc100_conf->q_ul_4g.num_qgroups +
			acc100_conf->q_ul_5g.num_qgroups +
			acc100_conf->q_dl_4g.num_qgroups +
			acc100_conf->q_dl_5g.num_qgroups;
	if (totalQgs > ACC100_NUM_QGRPS) {
		printf("ERROR: Number of Qgroups %d > %d\n",
				totalQgs, ACC100_NUM_QGRPS);
		return -1;
	}
	if (acc100_conf->num_vf_bundles > ACC100_NUM_VFS) {
		printf("ERROR: Number of VFs bundle %d > %d\n",
				acc100_conf->num_vf_bundles, ACC100_NUM_VFS);
		return -1;
	}

	if (acc100_queue_range_check(acc100_conf->q_ul_4g) != 0)
		return -1;
	if (acc100_queue_range_check(acc100_conf->q_dl_4g) != 0)
		return -1;
	if (acc100_queue_range_check(acc100_conf->q_ul_5g) != 0)
		return -1;
	if (acc100_queue_range_check(acc100_conf->q_dl_5g) != 0)
		return -1;
	return 0;
}

int
acc100_parse_conf_file(const char *file_name, struct acc100_conf *acc100_conf)
{
	int ret;

	set_default_config(acc100_conf);

	ret = ini_parse(file_name, acc100_handler, acc100_conf);

	if (ret != 0) {
		printf("ERROR: Config file parser error\n");
		set_default_config(acc100_conf);
		return -1;
	}

	ret = acc100_range_check(acc100_conf);
	if (ret != 0) {
		printf("ERROR: Parameter out of range\n");
		set_default_config(acc100_conf);
		return -1;
	}

	return 0;
}
