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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>

#define BB_ACC_MAX_LOG_FILE_NAME 500
struct bb_acc_logCtl {
	int level;
	FILE *main_fp;
	FILE *secondary_fp;
	uint64_t maxSize;
	int vfiopciMode;
};

const char *logStr[] = {
	"ERR",
	"WARN",
	"INFO",
	"DEBUG"
};

static struct bb_acc_logCtl logCtl;

static int
bb_acc_vlog(int level, int log_type, const char *format, va_list ap)
{
	int ret;
	if (!logCtl.vfiopciMode) {
		if (level > logCtl.level)
			return 0;
		ret = vprintf(format, ap);
	} else {
		FILE *fp = logCtl.main_fp;
		if (log_type != 0)
			fp = logCtl.secondary_fp;

		if (level > logCtl.level)
			return 0;

		if (fp == NULL)
			return -1;

		if (ftell(fp) > logCtl.maxSize)
			fseek(fp, 0, SEEK_SET);

		ret = vfprintf(fp, format, ap);
		fflush(fp);
	}
	return ret;
}

int
bb_acc_log(int level, int log_type, const char *format, ...)
{
	va_list ap;
	int ret;

	va_start(ap, format);
	ret = bb_acc_vlog(level, log_type, format, ap);
	va_end(ap);

	return ret;
}

int
bb_acc_logInit(char *main_file_name, char *resp_file_name, uint64_t size, int level,
		int vfiopciMode)
{
	FILE *fp = NULL, *fp_2 = NULL;

	if (!vfiopciMode) {
		/* igb_uio mode. */
		logCtl.main_fp = fp;
		logCtl.secondary_fp = fp;
		logCtl.maxSize = size; /* having no use. */
		logCtl.level = level;
		logCtl.vfiopciMode = vfiopciMode;
	} else {
		if (logCtl.main_fp != NULL)
			return 0;

		fp = fopen(main_file_name, "r+");
		if (fp == NULL) {
			fp = fopen(main_file_name, "w+");
			if (fp == NULL) {
				perror("Log file open failed");
				return -1;
			}
		}
		fp_2 = fopen(resp_file_name, "r+");
		if (fp_2 == NULL) {
			fp_2 = fopen(resp_file_name, "w+");
			if (fp_2 == NULL) {
				perror("Log file open failed");
				fclose(fp);
				return -1;
			}
		}

		logCtl.main_fp = fp;
		logCtl.secondary_fp = fp_2;
		logCtl.maxSize = size;
		logCtl.level = level;
		logCtl.vfiopciMode = vfiopciMode;
		fseek(fp, 0, SEEK_END);
		fseek(fp_2, 0, SEEK_END);
	}
	return 0;
}

void
bb_acc_logExit()
{
	if (logCtl.main_fp == NULL)
		return;

	fclose(logCtl.main_fp);
	fclose(logCtl.secondary_fp);
}

void
bb_acc_reset_logFile(char *file_name, int log_type)
{
	char sysCmd[1024];
	memset(sysCmd, 0, sizeof(sysCmd));
	if (logCtl.main_fp == NULL)
		return;
	sprintf(sysCmd, "sudo truncate -s 0 %s", file_name);
	if (system(sysCmd))
		perror("Failed to clear the log file");
	if (log_type == 0)
		fseek(logCtl.main_fp, 0, SEEK_SET);
	else
		fseek(logCtl.secondary_fp, 0, SEEK_SET);
}
