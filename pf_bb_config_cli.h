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

#ifndef __PF_BB_CONFIG_CLI_H__
#define __PF_BB_CONFIG_CLI_H__

#define UNIX_CHANNEL_PATH  "/tmp/pf_bb_config"
#define UNIX_CHANNEL_FILE_LEN  64

#define NO_WAIT_ON_RESP 0
#define WAIT_ON_RESP    1

/*
 * enum defining different commands supported
 * by pf_bb_config_cli.
 * HELP_CMD_ID		-->To displays argument list, NULL if nor required.
 * RESET_MODE_CMD_ID	-->To set reset mode of ACC devices (pf_flr / cluster_reset).
 * AUTO_RESET_CMD_ID	-->To auto reset ACC devices (on / off).
 * CLEAR_LOG_CMD_ID	-->To clear the previous content of logfile file.
 * EXIT_APP_CMD_ID	-->To gracefully shutdown the application.
 * REG_DUMP_CMD_ID	-->To dump all the register status of the ACC device (DEVICE_ID).
 * RECONFIG_ACC_CMD_ID	-->To reconfigure the ACC device (DEVICE_ID).
 * MM_READ_CMD_ID	-->To read/write to a register.
 * DEVICE_DATA_CMD_ID	-->To dump telemetry data.
 */

enum {
	HELP_CMD_ID		= 0x1,
	RESET_MODE_CMD_ID	= 0x2,
	AUTO_RESET_CMD_ID	= 0x3,
	CLEAR_LOG_CMD_ID	= 0x4,
	EXIT_APP_CMD_ID		= 0x5,
	REG_DUMP_CMD_ID		= 0x6,
	RECONFIG_ACC_CMD_ID	= 0x7,
	MM_READ_CMD_ID		= 0x8,
	DEVICE_DATA_CMD_ID	= 0x9
};

#define CLI_CMD_MAX_LEN 64
struct cmdDef {
	int  id;
	char cmd[CLI_CMD_MAX_LEN];
	void (*help)(void);
	int  (*parse)(int argc, char *argv[], void *ci);
	int  (*send)(struct cmdDef *, void *);
	int  (*resp)(struct cmdDef *, void *);
	void (*print)(struct cmdDef *, void *);
	int  (*exec)(struct cmdDef *, void *);
	char *short_help;
};

#define NULL_HELP_FP   NULL
#define NULL_PARSE_FP  NULL
#define NULL_SEND_FP   NULL
#define NULL_RESP_FP   NULL
#define NULL_PRINT_FP  NULL
#define NULL_EXEC_FP   NULL

struct help_req {
	void (*cmd_help)(void);
};

#define RESET_MODE_CLUSTER_LEVEL 0
#define RESET_MODE_FLR 1
struct reset_mode_req {
	unsigned int mode;
} __attribute__((__packed__));

#define AUTO_RESET_OFF 0
#define AUTO_RESET_ON 1
struct auto_reset_req {
	unsigned int mode;
} __attribute__((__packed__));

#define DEVICE_ID_ACC100	0x0D5C
#define DEVICE_ID_VRB1	0x57C0

struct reg_dump_req {
	unsigned int device_id;
} __attribute__((__packed__));

struct reg_dump_resp {
	int code;
} __attribute__((__packed__));

#define MM_READ_REG_READ	0
#define MM_READ_REG_WRITE	1
struct mm_read_req {
	/* reg_op_flag describes read/write operation*/
	unsigned int reg_op_flag;
	unsigned int reg_rw_address;
	unsigned int write_payload;
} __attribute__((__packed__));

#define CMD_REQ(ci) (&(((struct cmdReq *)ci)->req))
struct cmdReq {
	short id;
	short len;
	void *priv;
	union {
		struct help_req help;
		struct reset_mode_req reset_mode_req;
		struct auto_reset_req auto_reset_req;
		struct reg_dump_req reg_dump_req;
		struct mm_read_req mm_read_req;
	} req;
};

struct cmdResp {
	short id;
	short len;
	unsigned int resp_code;
	union {
		struct reg_dump_resp reg_dump_resp;
	} resp;
};

#endif  /* __PF_BB_CONFIG_CLI_H__ */
