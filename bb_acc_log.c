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
	FILE *fp;
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
bb_acc_vlog(int level, const char *format, va_list ap)
{
	int ret;
	if (!logCtl.vfiopciMode) {
		if (level > logCtl.level)
			return 0;
		ret = vprintf(format, ap);
	} else {
		FILE *fp = logCtl.fp;

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
bb_acc_log(int level, const char *format, ...)
{
	va_list ap;
	int ret;

	va_start(ap, format);
	ret = bb_acc_vlog(level, format, ap);
	va_end(ap);

	return ret;
}

int
bb_acc_logInit(char *file_name, uint64_t size, int level, int vfiopciMode)
{
	FILE *fp = NULL;

	if (!vfiopciMode) {
		/*igb_uio mode*/
		logCtl.fp = fp;
		logCtl.maxSize = size;/*having no use*/
		logCtl.level = level;
		logCtl.vfiopciMode = vfiopciMode;
	} else {
		if (logCtl.fp != NULL)
			return 0;

		fp = fopen(file_name, "r+");
		if (fp == NULL) {
			fp = fopen(file_name, "w+");
			if (fp == NULL) {
				perror("Log file open failed");
				return -1;
			}
		}

		logCtl.fp = fp;
		logCtl.maxSize = size;
		logCtl.level = level;
		logCtl.vfiopciMode = vfiopciMode;
		fseek(fp, 0, SEEK_END);
	}
	return 0;
}

void
bb_acc_logExit()
{
	if (logCtl.fp == NULL)
		return;

	fclose(logCtl.fp);
}

void
bb_acc_reset_logFile(char *file_name)
{
	char sysCmd[1024];
	memset(sysCmd, 0, sizeof(sysCmd));
	if (logCtl.fp == NULL)
		return;
	sprintf(sysCmd, "sudo truncate -s 0 %s", file_name);
	if (system(sysCmd))
		perror("Failed to clear the log file");
	fseek(logCtl.fp, 0, SEEK_SET);
}
