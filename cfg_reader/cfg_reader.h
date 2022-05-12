
#ifndef __CFG_READER_H__
#define __CFG_READER_H__

/* Make this header file easier to include in C++ code */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define MAX_SECTION_LEN 50
#define MAX_NAME_LEN 50
#define CFG_MAX_LINE_LEN 200

typedef int (*cfg_handler)(void *user, const char *section,
			const char *name, const char *value);


int cfg_parse(const char *filename, cfg_handler handler, void *user);
bool cfg_file_check_path_safety(const char *cfg_filename);

#ifdef __cplusplus
}
#endif

#endif /* __CFG_READER_H__ */
