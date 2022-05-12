#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "cfg_reader.h"

/*rstrip function, remove right space*/
static char*
rstrip(char *s)
{
	char *p = s + strlen(s);
	while (p > s) {
		p--;
		if (*p == ' ' || *s == '\t' || *p == '\n')
			*p = '\0';
		else
			break;
	}
	return s;
}

/*lstrip function, remove left space*/
static char*
lstrip(char *s)
{
	while (*s && (*s == ' ' || *s == '\t' || *s == '\n'))
		s++;
	return s;
}



/*forward search targets location and pointer move to it*/
static char*
jump_to_targets_location(char *s, char *targets)
{
	if (targets == NULL) {
		while (*s && (*s != '\0'))
			s++;
		return s;
	}
	while (*s && (*s != '\0')) {
		if (strchr(targets, *s))
			break;
		s++;
	}
	return s;
}

/*cfg parse main workflow*/
int
cfg_parse_contents(void *contents, cfg_handler handler, void *user)
{

	char line[CFG_MAX_LINE_LEN];
	int max_line_len = CFG_MAX_LINE_LEN;

	char section[MAX_SECTION_LEN] = "";
	char name[MAX_NAME_LEN] = "";

	char *left;
	char *right;
	char *cur_name;
	char *value;
	int line_num = 0;
	int error_line_index = 0;

	while (fgets(line, max_line_len, contents) != NULL) {

		line_num++;
		left = line;
		/* skip UTF-8 BOM char sequence if it exists */
		left = (line_num == 1 && left[0] == 0xEF && left[1] == 0xBB
				&& left[2] == 0xBF) ? (left + 3) : left;

		left = lstrip(rstrip(left));

		if (*left == ';' || *left == '#') {
			/* skip start-of-line comment */
			continue;
		} else if (*left == '[') {
			/* parse "section" */
			right = jump_to_targets_location(left + 1, "]");
			if (*right == ']') {
				*right = '\0';
				strncpy(section, left + 1, MAX_SECTION_LEN - 1);
				section[MAX_SECTION_LEN - 1] = '\0';
			} else {
				printf("error, no [ exists\n");
				error_line_index = line_num;
				break;
			}
		} else if (*left) {
			right = jump_to_targets_location(left, "=:");
			if (*right == '=' || *right == ':') {
				/* parse "name" and "value" */
				*right = '\0';
				cur_name = rstrip(left);
				strncpy(name, cur_name, MAX_NAME_LEN - 1);
				name[MAX_NAME_LEN - 1] = '\0';
				value = right + 1;
				value = lstrip(rstrip(value));
				if (handler(user, section, name, value) != 1) {
					error_line_index = line_num;
					break;
				}
			} else {
				printf("error, no \":\" or \"=\" exists\n");
				error_line_index = line_num;
				break;
			}
		}
	}

	return error_line_index;
}

/*cfg parse interface*/
int
cfg_parse(const char *filename, cfg_handler handler, void *user)
{

	FILE *file;
	int error_line_index;
	file = fopen(filename, "r");
	if (!file) {
		printf("error, cannot load cfg files");
		return -1;
	}
	error_line_index = cfg_parse_contents(file, handler, user);
	fclose(file);
	return error_line_index;
}

/*check config path's safety, return true when found unsafe path*/
bool
cfg_file_check_path_safety(const char *cfg_filename) {
	if (cfg_filename == NULL) {
		/* check if the config path is "NULL" */
		return true;
	}
	int offset = 0;
	int len = strlen(cfg_filename) + 1;
	if (len < 3)
		return false;
	/* check if the config path includes "../" */
	for (offset = 0; offset <= len - 3; offset++) {
		int a = (*(cfg_filename + offset) == '.');
		int b = (*(cfg_filename + offset + 1) == '.');
		int c = (*(cfg_filename + offset + 2) == '/');
		if (a && b && c)
			return true;
	}
	return false;
}
