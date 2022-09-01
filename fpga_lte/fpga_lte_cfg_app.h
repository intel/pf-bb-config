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

#ifndef _FPGA_LTE_CFG_APP_H_
#define _FPGA_LTE_CFG_APP_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include <errno.h>

/**< Number of Virtual Functions FGPA LTE FEC supports */
#define FPGA_LTE_FEC_NUM_VFS 8

struct
fpga_lte_fec_conf {
	/**< 1 if PF is used for dataplane, 0 for VFs */
	bool pf_mode_en;
	/**< Number of UL queues per VF */
	uint8_t vf_ul_queues_number[FPGA_LTE_FEC_NUM_VFS];
	/**< Number of DL queues per VF */
	uint8_t vf_dl_queues_number[FPGA_LTE_FEC_NUM_VFS];
	/**< UL bandwidth. Needed for schedule algorithm */
	uint8_t ul_bandwidth;
	/**< DL bandwidth. Needed for schedule algorithm */
	uint8_t dl_bandwidth;
	/**< UL Load Balance */
	uint8_t ul_load_balance;
	/**< DL Load Balance */
	uint8_t dl_load_balance;
	/**< FLR timeout value */
	uint16_t flr_time_out;
};

#endif /* _FPGA_CFG_APP_H_ */
