/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#include "ptlib_file.h"

int file_copy(char *copy_from_path, char *copy_to_path) {
    int result = 1;
	const unsigned int read_size = 256;
	unsigned char read_ch[read_size];
	int tot_bytes = 0;
	size_t read_ret;
	size_t write_ret;

    errno = 0;

    FILE* to_fptr = fopen(copy_to_path, "wb");
    if(to_fptr == NULL) {
        output(ERROR, "%s: %s\n", copy_to_path, strerror( errno ));
        goto END;
    }

    FILE* from_fptr = fopen(copy_from_path, "rb");
    if(from_fptr == NULL) {
        output(ERROR, "%s: %s\n", copy_from_path, strerror( errno ));
        fclose(to_fptr);
        goto END;
    }

	while(1) {
		read_ret = fread(read_ch, sizeof *read_ch, read_size, from_fptr);
		tot_bytes += read_ret;
		write_ret = fwrite(read_ch, sizeof *read_ch, read_ret, to_fptr);
		if(write_ret != read_ret) {
			output(ERROR, "Error writing %s: %s\n",
				copy_to_path, strerror(errno));
			break;
		}
		if(write_ret == EOF) {
			output(DEBUG, "Error writing %s: %s\n",
				copy_to_path, strerror(errno));
			break;
		}
		if(read_ret != read_size) {
			if(feof(from_fptr)) {
				if(errno != 0) {
					output(ERROR, "%s\n", strerror(errno));
				} else {
					result = 0;
				}
			} else {
				output(DEBUG, "Unexpected file read error: %s\n",
					strerror(errno));
			}
			break;
		}
	}
	output(DEBUG, "Read %d bytes\n", tot_bytes);
    fclose(to_fptr);
    fclose(from_fptr);

END:
    return result;
}

#define FILE_INSERT_TMP_FILE "/ptmfg_csv_temp-XXXXXX"
int file_insert(char *source_file, char *working_dir, char *regex_str,
		char *string_to_insert){
	FILE *source_fp = NULL;
	char line[2048];
	char regexec_error_message[100];
	char *tmp_file_path = NULL;
	char *source_file_path = NULL;
	regex_t regex;
	int rc = -1;
	int regex_resp;
	int filedes = -1;
	bool error = false;

	output(DEBUG, "Start: %s.\n", __func__);
	int source_file_path_len = strlen(source_file) + strlen(working_dir) + 1;
	source_file_path = calloc(source_file_path_len, 1);
	if (NULL == source_file_path) {
		output(ERROR, "Failed to allocate memory\n");
		rc = 1;
		goto END;
	}
	strlcpy(source_file_path, working_dir, source_file_path_len);
	strlcat(source_file_path, source_file, source_file_path_len);

	regex_resp = regcomp(&regex, regex_str, 0);
	if (0 != regex_resp) {
		output(ERROR, "Could not compile regular expression.\n");
		output(ERROR, "Source file (%s) will not be modified.\n",
				source_file_path);
		rc = 1;
		goto END;
	}

	int tmp_file_path_len = strlen(FILE_INSERT_TMP_FILE)
			+ strlen(working_dir) + 1;
	tmp_file_path = calloc(tmp_file_path_len, 1);
	if (NULL == tmp_file_path) {
		output(ERROR, "Failed to allocate memory\n");
		rc = 1;
		goto END;
	}
	strlcpy(tmp_file_path, working_dir, tmp_file_path_len);
	strlcat(tmp_file_path, FILE_INSERT_TMP_FILE, tmp_file_path_len);
	errno = 0;
	filedes = mkstemp(tmp_file_path);
	if(filedes < 0) {
		output(ERROR, "Creation of tmp file failed with error [%s]\n",
				strerror(errno));
		output(ERROR, "Source file (%s) will not be modified.\n",
				source_file_path);
		rc = 1;
		goto END;
	}
	source_fp = fopen(source_file_path, "r");
	if(NULL != source_fp) {
		while(fgets(line, sizeof(line), source_fp) != NULL) {
			regex_resp = regexec(&regex, line, 0, NULL, 0);
			if (0 == regex_resp) {
				errno = 0;
				if(-1 == write(filedes, line, strlen(line))) {
					output(ERROR, "Write failed with error [%s]\n",
							 strerror(errno));
					output(ERROR, "Source file (%s) will not be modified.\n",
							source_file_path);
					error = true;
					break;
				}
				errno = 0;
				if(-1 == write(filedes, string_to_insert,
						strlen(string_to_insert))) {
					output(ERROR, "Write failed with error [%s]\n",
							 strerror(errno));
					output(ERROR, "Source file (%s) will not be modified.\n",
							source_file_path);
					error = true;
					break;
				 }
			}
			else if (regex_resp == REG_NOMATCH) {
				errno = 0;
				if(-1 == write(filedes, line, strlen(line))) {
					output(ERROR, "Write failed with error [%s]\n",
							 strerror(errno));
					output(ERROR, "Source file (%s) will not be modified.\n",
							source_file_path);
					error = true;
					break;
				}
			}
			else {
				regerror(regex_resp, &regex, regexec_error_message,
						sizeof(regexec_error_message));
				output(ERROR,
						"Regular expression search encountered an error. %s.\n",
						regexec_error_message);
				output(ERROR, "Source file (%s) will not be modified.\n",
						source_file_path);
				error = true;
				break;
			}
		}
		fclose(source_fp);
	} else {
		output(DEBUG, "%s: Could not open log file for reading (%s).\n",
				__func__, source_file_path);
		error = true;
	}
	close(filedes);
	regfree(&regex);
	if (error) {
		unlink(tmp_file_path);
		rc = 1;
		goto END;
	}
	unlink(source_file_path);
	if (rename(tmp_file_path, source_file_path) < 0) {
		output(ERROR, "Renaming temporary file failed. Temporary file\n");
		output(ERROR, "may be found at: %s\n", tmp_file_path);
		rc = 1;
		goto END;
	}
	unlink(tmp_file_path);

END:
	free(tmp_file_path);
	free(source_file_path);
	return rc;
}

Poll_Status fpoll_inbound_data(FILE* fptr, time_t timeout)
{
	if (fptr == NULL) {
		output(ERROR, "%s: Given a NULL file handle argument.\n", __func__);
		return POLL_STATUS_ERROR;
	}

	int fd = fileno(fptr);

	fd_set set;
	FD_ZERO(&set);
	FD_SET(fd, &set);
	struct timeval timeout_tv = { .tv_sec = 0, .tv_usec = timeout };
	int select_rc = select(fd + 1, &set, NULL, NULL, &timeout_tv);
	if (select_rc == -1) {
		output(ERROR, "%s: A problem occurred while trying to read from the "
				"sysfs node. %s [%d]\n", __func__, strerror(errno), errno);
		return POLL_STATUS_ERROR;
	} else if (select_rc == 0) {
		output(DEBUG, "Polling timed-out for incoming data.\n");
		return POLL_STATUS_TIMEOUT;
	} else {
		return POLL_STATUS_GOT_DATA;
	}
}
