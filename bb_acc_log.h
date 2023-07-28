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

#ifndef __BB_ACC_LOG_H__
#define __BB_ACC_LOG_H__

#include <time.h>

typedef enum {
	ERR = 0,
	WARN,
	INFO,
	DEBUG
} bb_acc_log_level;

extern const char *logStr[];

static char *timestamp()
{
	char *time_s;
	time_t now = time(NULL);
	const struct tm *gtm = gmtime(&now);
	if (gtm == NULL)
		return NULL;

	time_s = asctime(gtm);
	if (time_s == NULL)
		return NULL;

	time_s[strlen(time_s)-1] = '\0';
	return time_s;
}

/* Default log file size is 2MB */
#define BB_ACC_MAX_LOG_FILE_SIZE (2 * 1024 * 1024)

/* Default log file path and name */
#define BB_ACC_DEFAULT_LOG_PATH "/var/log"
#define BB_ACC_LOG_FILE_LEN  128

#define BB_ACC_LOG_MAIN 0
#define BB_ACC_LOG_RESP 1

#define BB_ACC_LOG_RAW(level, fmt, args...) \
	bb_acc_log(level, BB_ACC_LOG_MAIN, "%s:%s:%s(): " fmt "\n", \
	timestamp(), logStr[level], __func__, ## args)

#define BB_ACC_LOG(level, fmt, args...) \
	bb_acc_log(level, BB_ACC_LOG_MAIN, "%s:%s:" fmt "\n", timestamp(), logStr[level], ## args)

#define LOG(level, fmt, args...) \
	({\
		if (level == DEBUG)\
			BB_ACC_LOG_RAW(level, fmt, ## args);\
		else\
			BB_ACC_LOG(level, fmt, ## args);\
	})

#define LOG_RESP(level, fmt, args...) \
	bb_acc_log(level, BB_ACC_LOG_RESP, "%s:%s:" fmt "\n", timestamp(), logStr[level], ## args)

#define LOG_RESP_END() \
	LOG_RESP(INFO, "-- End of Response --")


extern int bb_acc_log(int level, int log_type, const char *format, ...);
extern int bb_acc_logInit(char *main_file_name, char *resp_file_name, uint64_t size,
		int level, int vfiopciMode);
extern void bb_acc_logExit(void);
extern void bb_acc_reset_logFile(char *file_name, int log_type);

#endif /* __BB_ACC_LOG_H__ */
