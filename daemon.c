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

#include <stdlib.h>
#include <stdint.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <strings.h>
#include <unistd.h>
#include <stdbool.h>
#include <limits.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <signal.h>

#include "bb_acc.h"
#include "bb_acc_log.h"

struct event_fd {
	int    fd[MAX_EVENTS];
	cbFp_t cbFp[MAX_EVENTS];
	void  *cbData[MAX_EVENTS];
	struct pollfd pfds[MAX_EVENTS];
	int    pollfd_count;
};

static struct event_fd eventFd;

struct cmdReq cmdReq_g;

static int is_sig_term_rcv;

static int reset_mode_exec(struct cmdDef *cmd, void *ci);
static int auto_reset_exec(struct cmdDef *cmd, void *ci);
static int clear_log_exec(struct cmdDef *cmd, void *ci);
static int exit_app_exec(struct cmdDef *cmd, void *ci);
static int reg_dump_exec(struct cmdDef *cmd, void *ci);
static int reconfigure_acc_exec(struct cmdDef *cmd, void *ci);
static int mem_read_exec(struct cmdDef *cmd, void *ci);
static int device_data_exec(struct cmdDef *cmd, void *ci);

struct cmdDef cmd_defs[] = {
	{
		RESET_MODE_CMD_ID, "reset_mode",
		NULL_HELP_FP,
		NULL_PARSE_FP,
		NULL_SEND_FP,
		NULL_RESP_FP,
		NULL_PRINT_FP,
		reset_mode_exec,
		"",
	},
	{
		AUTO_RESET_CMD_ID, "auto_reset",
		NULL_HELP_FP,
		NULL_PARSE_FP,
		NULL_SEND_FP,
		NULL_RESP_FP,
		NULL_PRINT_FP,
		auto_reset_exec,
		"",
	},
	{
		CLEAR_LOG_CMD_ID, "clear_log",
		NULL_HELP_FP,
		NULL_PARSE_FP,
		NULL_SEND_FP,
		NULL_RESP_FP,
		NULL_PRINT_FP,
		clear_log_exec,
		"",
	},
	{
		EXIT_APP_CMD_ID, "exit_app",
		NULL_HELP_FP,
		NULL_PARSE_FP,
		NULL_SEND_FP,
		NULL_RESP_FP,
		NULL_PRINT_FP,
		exit_app_exec,
		"",
	},
	{
		REG_DUMP_CMD_ID, "reg_dump",
		NULL_HELP_FP,
		NULL_PARSE_FP,
		NULL_SEND_FP,
		NULL_RESP_FP,
		NULL_PRINT_FP,
		reg_dump_exec,
		"",
	},
	{
		RECONFIG_ACC_CMD_ID, "reconfigure_acc",
		NULL_HELP_FP,
		NULL_PARSE_FP,
		NULL_SEND_FP,
		NULL_RESP_FP,
		NULL_PRINT_FP,
		reconfigure_acc_exec,
		"",
	},
	{
		MM_READ_CMD_ID, "mm_read",
		NULL_HELP_FP,
		NULL_PARSE_FP,
		NULL_SEND_FP,
		NULL_RESP_FP,
		NULL_PRINT_FP,
		mem_read_exec,
		"",
	},
	{
		DEVICE_DATA_CMD_ID, "device_data",
		NULL_HELP_FP,
		NULL_PARSE_FP,
		NULL_SEND_FP,
		NULL_RESP_FP,
		NULL_PRINT_FP,
		device_data_exec,
		"",
	},
	{  /* must be at the end */
		-1, "",
		NULL_HELP_FP,
		NULL_PARSE_FP,
		NULL_SEND_FP,
		NULL_RESP_FP,
		NULL_PRINT_FP,
		NULL_EXEC_FP,
		""
	}
};
#define NO_OF_CMDS (int)((sizeof(cmd_defs)/sizeof(struct cmdDef))-1)

int
unix_channel_srv_init(hw_device *dev)
{
	int	fd;
	int	rc;
	struct sockaddr_un serveraddr;
	char unix_channel_file[UNIX_CHANNEL_FILE_LEN];

	LOG(DEBUG, "cli channel init");

	fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		LOG(ERR, "cli channel: socket() open failed");
		return -1;
	}

	sprintf(unix_channel_file, "%s.%s.sock", UNIX_CHANNEL_PATH, dev->pci_address);
	memset(&serveraddr, 0, sizeof(serveraddr));
	serveraddr.sun_family = AF_UNIX;
	strcpy(serveraddr.sun_path, unix_channel_file);

	rc = bind(fd, (struct sockaddr *)&serveraddr, SUN_LEN(&serveraddr));
	if (rc < 0) {
		LOG(ERR, "cli channel: bind() failed");
		close(fd);
		return -1;
	}

	/* supports only one client connection at a time */
	rc = listen(fd, 1);
	if (rc < 0) {
		LOG(ERR, "cli channel: listen() failed");
		close(fd);
		return -1;
	}

	LOG(DEBUG, "Done");

	return fd;
}

static int
unix_channel_srv_rx(int fd, struct cmdReq *req)
{
	int rc;
	int client_fd;

	LOG(DEBUG, "data received on cli channel");

	client_fd = accept(fd, NULL, NULL);
	if (client_fd < 0) {
		LOG(ERR, "cli channel: accept() failed");
		return -1;
	}
	LOG(DEBUG, "client connected fd: %d", client_fd);

	rc = recv(client_fd, req, sizeof(struct cmdReq), 0);
	if (rc < 0) {
		LOG(ERR, "cmd request recv() failed");
		return 0;
	}

	LOG(DEBUG, "Req id: %04x, len: %d", req->id, req->len);

	return 0;
}

void event_init_fd(void)
{
	int i;

	LOG(DEBUG, "Event init fd");

	for (i = 0; i < MAX_EVENTS; i++)
		eventFd.fd[i] = -1;
}

int event_add_fd(int fd, cbFp_t cbFp,  void *cbData)
{
	int i;

	if (eventFd.pollfd_count >= MAX_EVENTS) {
		LOG(ERR, "failed to add event fd");
		return -1;
	}

	for (i = 0; i < MAX_EVENTS; i++) {
		if (eventFd.fd[i] != -1)
			continue;

		eventFd.fd[i] = fd;
		eventFd.cbFp[i] = cbFp;
		eventFd.cbData[i] = cbData;
		eventFd.pollfd_count++;
		LOG(DEBUG, "location: %d, pollfd_count=%d", i, eventFd.pollfd_count);
		return 0;
	}

	LOG(ERR, "Event fd add failed");
	return -1;
}

int event_del_fd(int fd)
{
	int i;

	for (i = 0; i < MAX_EVENTS; i++) {
		if (eventFd.fd[i] == fd) {
			eventFd.fd[i] = -1;
			eventFd.cbFp[i] = NULL;
			eventFd.cbData[i] = NULL;
			eventFd.pollfd_count--;
			LOG(DEBUG, "location: %d, pollfd_count=%d", i, eventFd.pollfd_count);
			return 0;
		}
	}

	LOG(ERR, "Event del failed");
	return -1;
}

static void event_set_pfds(void)
{
	int i;

	for (i = 0; i < MAX_EVENTS; i++) {
		eventFd.pfds[i].fd = eventFd.fd[i];
		eventFd.pfds[i].events = POLLIN;
	}
}

static void event_process_fds(void)
{
	int i;

	for (i = 0; i < MAX_EVENTS; i++) {
		if ((eventFd.pfds[i].fd != -1) && (eventFd.pfds[i].revents & POLLIN)) {
			eventFd.pfds[i].revents = 0;
			if (eventFd.cbFp[i])
				eventFd.cbFp[i](eventFd.pfds[i].fd, eventFd.cbData[i]);
		}
	}
}

static int
get_cmd_by_id(int cmd_id, struct cmdDef **cd)
{
	int i;

	for (i = 0; i < NO_OF_CMDS; i++) {
		if (cmd_id == cmd_defs[i].id)
			break;
	}

	if (cmd_defs[i].id == -1) {
		LOG(ERR, "command id: %d is not supported", cmd_id);
		return -1;
	}

	*cd = &cmd_defs[i];

	LOG(DEBUG, "found command: %s, id: %d", cmd_defs[i].cmd, cmd_defs[i].id);

	return 0;
}

void cli_channel_event_processor(int fd, void *data)
{
	struct cmdReq *req = (struct cmdReq *)&cmdReq_g;
	struct cmdDef *cmd;

	if (data == NULL) {
		LOG(ERR, "event data is null");
		return;
	}
	if (!unix_channel_srv_rx(fd, req)) {
		if (get_cmd_by_id(req->id, &cmd) == -1)
			return;
		if (cmd->exec != NULL) {
			req->priv = data;
			cmd->exec(cmd, (void *)req);
		}
	}
}

int cli_channel_init(hw_device *dev)
{
	int fd;

	fd = unix_channel_srv_init(dev);
	if (fd < 0)
		return -1;

	if (event_add_fd(fd, cli_channel_event_processor, dev) < 0) {
		close(fd);
		return -1;
	}

	return fd;
}

/* Check events on in the given fds list
*  blocks on events
*/
static void
event_processor(hw_device *dev)
{
	char unix_channel_file[UNIX_CHANNEL_FILE_LEN];
	char sysCmd[1024];
	int r;
	while (!is_sig_term_rcv) {

		event_set_pfds();

		LOG(DEBUG, "Waiting on poll...");

		/* Waits on list of events */
		poll(eventFd.pfds, MAX_EVENTS, -1);

		LOG(DEBUG, "poll() unblocked, check events");

		/* process all events that are set */
		event_process_fds();
	}
	if (is_sig_term_rcv == 1) {
		memset(sysCmd, 0, sizeof(sysCmd));
		sprintf(unix_channel_file, "%s.%s.sock", UNIX_CHANNEL_PATH, dev->pci_address);
		sprintf(sysCmd, "rm -fr %s", unix_channel_file);
		r = system(sysCmd);
		LOG(DEBUG, "system cmd returns = %d\n", r);
	}

}

void
daemonize(hw_device *dev)
{
	pid_t pid, sid;

	(void) signal(SIGTERM, sig_fun);
	(void) signal(SIGKILL, sig_fun);
	(void) signal(SIGABRT, sig_fun);
	(void) signal(SIGSEGV, sig_fun);
	(void) signal(SIGBUS, sig_fun);
	(void) signal(SIGILL, sig_fun);

	pid = fork();
	if (pid < 0)
		exit(EXIT_FAILURE);

	/* exit the parent */
	if (pid > 0)
		exit(EXIT_SUCCESS);

	umask(0);

	/* do not let the child to become orphan */
	sid = setsid();
	if (sid < 0)
		exit(EXIT_FAILURE);

	close(STDIN_FILENO);
	close(STDOUT_FILENO);
	close(STDERR_FILENO);

	if (cli_channel_init(dev) < 0) {
		LOG(ERR, "cli channel init failed");
		return;
	}

	LOG(DEBUG, "Event processor started");

	/* blocks on events */
	event_processor(dev);

}

static int
reset_mode_exec(struct cmdDef *cmd, void *ci)
{
	struct cmdReq *cmd_req = (struct cmdReq *)ci;
	struct reset_mode_req *reset_mode = (struct reset_mode_req *)CMD_REQ(ci);
	const char *reset_mode_str[2] = {"cluster_reset", "pf_flr"};


	if ((reset_mode->mode != RESET_MODE_CLUSTER_LEVEL) &&
			(reset_mode->mode != RESET_MODE_FLR)) {
		LOG(ERR, "Unknown reset_mode request:%d", reset_mode->mode);
		return -1;
	}
	LOG(INFO, "reset_mode set to : %s", reset_mode_str[reset_mode->mode]);

	update_reset_mode(cmd_req->priv, reset_mode->mode);

	return 0;
}

static int
auto_reset_exec(struct cmdDef *cmd, void *ci)
{
	struct cmdReq *cmd_req = (struct cmdReq *)ci;
	struct auto_reset_req *auto_reset = (struct auto_reset_req *)CMD_REQ(ci);
	const char *auto_reset_str[2] = {"off", "on"};

	if ((auto_reset->mode != AUTO_RESET_OFF) &&
			(auto_reset->mode != AUTO_RESET_ON)) {
		LOG(ERR, "Unknown auto_reset mode: %d", auto_reset->mode);
		return -1;
	}
	LOG(INFO, "Auto reset set to : %s", auto_reset_str[auto_reset->mode]);

	auto_reset_mode(cmd_req->priv, auto_reset->mode);

	return 0;
}

static int
clear_log_exec(struct cmdDef *cmd, void *ci)
{
	struct cmdReq *cmd_req = (struct cmdReq *)ci;

	LOG(INFO, "clear_log command received");

	clear_log_file(cmd_req->priv);

	return 0;
}
static int
exit_app_exec(struct cmdDef *cmd, void *ci)
{
	struct cmdReq *cmd_req = (struct cmdReq *)ci;

	LOG(INFO, "exit_app command received");

	exit_app_mode(cmd_req->priv);

	return 0;
}

static int
reg_dump_exec(struct cmdDef *cmd, void *ci)
{
	struct cmdReq *cmd_req = (struct cmdReq *)ci;
	struct reg_dump_req *reg_dump = (struct reg_dump_req *)CMD_REQ(ci);

	LOG(INFO, "reg_dump command received,  device_id : 0x%x", reg_dump->device_id);

	acc_reg_dump(cmd_req->priv, reg_dump->device_id);

	return 0;
}

static int
reconfigure_acc_exec(struct cmdDef *cmd, void *ci)
{
	struct cmdReq *cmd_req = (struct cmdReq *)ci;

	LOG(INFO, "reconfigure_acc command received");

	bb_acc_dev_reset_and_reconfig(cmd_req->priv);

	return 0;
}

static int
mem_read_exec(struct cmdDef *cmd, void *ci)
{
	struct cmdReq *cmd_req = (struct cmdReq *)ci;
	struct mm_read_req *mm_read = (struct mm_read_req *)CMD_REQ(ci);

	LOG(INFO, "mm_read command reveived");
	LOG(DEBUG, "mm_read: reg_op_flag	= 0x%x\n", mm_read->reg_op_flag);
	LOG(DEBUG, "mm_read: reg_rw_address	= 0x%x\n", mm_read->reg_rw_address);
	LOG(DEBUG, "mm_read: write_payload	= 0x%x\n", mm_read->write_payload);

	acc_mem_read(cmd_req->priv, mm_read->reg_op_flag,
			mm_read->reg_rw_address, mm_read->write_payload);

	return 0;
}

static int
device_data_exec(struct cmdDef *cmd, void *ci)
{
	struct cmdReq *cmd_req = (struct cmdReq *)ci;

	LOG(INFO, "device_data command received");

	acc_device_data(cmd_req->priv);

	return 0;
}

void sig_fun(int sig)
{
	uint8_t loopCnt = 0;
	uint16_t fdCnt = eventFd.pollfd_count;
	LOG(DEBUG, "Signal Received with sigNum = %u", sig);
	while (loopCnt < fdCnt) {
		/* Close all the communications */
		close(eventFd.fd[eventFd.pollfd_count]);
		eventFd.pollfd_count--;
		loopCnt++;
	}
	/* set "is_sig_term_rcv = 1"; so that event_processor exits */
	is_sig_term_rcv = 1;
}
