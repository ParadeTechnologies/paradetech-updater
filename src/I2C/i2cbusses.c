/*
 * Copyright (c) Parade Technologies, Ltd. 2023.
 */

#define _BSD_SOURCE 1

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/param.h>
#include <sys/ioctl.h>
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <dirent.h>
#include <fcntl.h>
#include <errno.h>
#include "i2cbusses.h"
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

enum adt { adt_dummy, adt_isa, adt_i2c, adt_smbus, adt_unknown };

struct adap_type {
	const char *funcs;
	const char* algo;
};

static struct adap_type adap_types[5] = {
	{ .funcs	= "dummy",
	  .algo		= "Dummy bus", },
	{ .funcs	= "isa",
	  .algo		= "ISA bus", },
	{ .funcs	= "i2c",
	  .algo		= "I2C adapter", },
	{ .funcs	= "smbus",
	  .algo		= "SMBus adapter", },
	{ .funcs	= "unknown",
	  .algo		= "N/A", },
};

static enum adt i2c_get_funcs(int i2cbus)
{
	unsigned long funcs;
	int file;
	char filename[20];
	enum adt ret;

	file = open_i2c_dev(i2cbus, filename, sizeof(filename), 1);
	if (file < 0)
		return adt_unknown;

	if (ioctl(file, I2C_FUNCS, &funcs) < 0)
		ret = adt_unknown;
	else if (funcs & I2C_FUNC_I2C)
		ret = adt_i2c;
	else if (funcs & (I2C_FUNC_SMBUS_BYTE |
			  I2C_FUNC_SMBUS_BYTE_DATA |
			  I2C_FUNC_SMBUS_WORD_DATA))
		ret = adt_smbus;
	else
		ret = adt_dummy;

	close(file);
	return ret;
}

static int rtrim(char *s)
{
	int i;

	for (i = strlen(s) - 1; i >= 0 && (s[i] == ' ' || s[i] == '\n'); i--)
		s[i] = '\0';
	return i + 2;
}

void free_adapters(struct i2c_adap *adapters)
{
	for (int i = 0; adapters[i].name; i++)
		free(adapters[i].name);
	free(adapters);
}

#define BUNCH	8

static struct i2c_adap *more_adapters(struct i2c_adap *adapters, int n)
{
	struct i2c_adap *new_adapters;

	new_adapters = realloc(adapters, (n + BUNCH) * sizeof(struct i2c_adap));
	if (!new_adapters) {
		free_adapters(adapters);
		return NULL;
	}
	memset(new_adapters + n, 0, BUNCH * sizeof(struct i2c_adap));

	return new_adapters;
}

struct i2c_adap *gather_i2c_busses(void)
{
	char s[120];
	struct dirent *de;
	struct dirent *dde;
	DIR *dir;
	DIR *ddir;
	FILE *f;
	char fstype[NAME_MAX];
	char sysfs[NAME_MAX+ 1];
	char n[NAME_MAX];
	int foundsysfs = 0;
	int count = 0;
	struct i2c_adap *adapters;

	adapters = calloc(BUNCH, sizeof(struct i2c_adap));
	if (!adapters)
		return NULL;

	if ((f = fopen("/proc/bus/i2c", "r"))) {
		while (fgets(s, 120, f)) {
			char *algo;
			char *name;
			char *type;
			char *all;
			int len_algo;
			int len_name;
			int len_type;
			int i2cbus;

			algo = strrchr(s, '\t');
			*(algo++) = '\0';
			len_algo = rtrim(algo);

			name = strrchr(s, '\t');
			*(name++) = '\0';
			len_name = rtrim(name);

			type = strrchr(s, '\t');
			*(type++) = '\0';
			len_type = rtrim(type);

			sscanf(s, "i2c-%d", &i2cbus);

			if ((count + 1) % BUNCH == 0) {
				adapters = more_adapters(adapters, count + 1);
				if (!adapters) {
					fclose(f);
					return NULL;
				}
			}

			all = malloc(len_name + len_type + len_algo);
			if (all == NULL) {
				free_adapters(adapters);
				fclose(f);
				return NULL;
			}
			adapters[count].nr = i2cbus;
			adapters[count].name = memcpy(all, name, len_name);
			adapters[count].funcs = memcpy(all + len_name, type, len_type);
			adapters[count].algo = memcpy(all + len_name + len_type, algo,
					len_algo);
			count++;
		}
		fclose(f);
		goto DONE;
	}

	if ((f = fopen("/proc/mounts", "r")) == NULL) {
		goto DONE;
	}
	while (fgets(n, NAME_MAX, f)) {
		sscanf(n, "%*[^ ] %[^ ] %[^ ] %*s\n", sysfs, fstype);
		if (strcasecmp(fstype, "sysfs") == 0) {
			foundsysfs++;
			break;
		}
	}
	fclose(f);
	if (! foundsysfs) {
		goto DONE;
	}

	strncat(sysfs, "/class/i2c-dev", NAME_MAX);
	if(!(dir = opendir(sysfs)))
		goto DONE;
	while ((de = readdir(dir)) != NULL) {
		if (!strcmp(de->d_name, "."))
			continue;
		if (!strcmp(de->d_name, ".."))
			continue;

		snprintf(n, NAME_MAX, "%s/%s/name", sysfs, de->d_name);
		f = fopen(n, "r");
		if(f == NULL) {
			snprintf(n, NAME_MAX, "%s/%s/device/name", sysfs, de->d_name);
			f = fopen(n, "r");
		}

		if(f == NULL) {
			snprintf(n, NAME_MAX, "%s/%s/device", sysfs, de->d_name);
			if(!(ddir = opendir(n)))
				continue;
			while ((dde = readdir(ddir)) != NULL) {
				if (!strcmp(dde->d_name, "."))
					continue;
				if (!strcmp(dde->d_name, ".."))
					continue;
				if (!strncmp(dde->d_name, "i2c-", 4)) {
					snprintf(n, NAME_MAX, "%s/%s/device/%s/name",
						sysfs, de->d_name, dde->d_name);
					if((f = fopen(n, "r")))
						goto FOUND;
				}
			}
		}

FOUND:
		if (f != NULL) {
			int i2cbus;
			enum adt type;
			char *px;

			px = fgets(s, 120, f);
			fclose(f);
			if (!px) {
				fprintf(stderr, "%s: read error\n", n);
				continue;
			}
			if ((px = strchr(s, '\n')) != NULL)
				*px = 0;
			if (!sscanf(de->d_name, "i2c-%d", &i2cbus))
				continue;
			if (!strncmp(s, "ISA ", 4)) {
				type = adt_isa;
			} else {
				type = i2c_get_funcs(i2cbus);
			}

			if ((count + 1) % BUNCH == 0) {
				adapters = more_adapters(adapters, count + 1);
				if (!adapters)
					return NULL;
			}

			adapters[count].nr = i2cbus;
			adapters[count].name = strdup(s);
			if (adapters[count].name == NULL) {
				free_adapters(adapters);
				return NULL;
			}
			adapters[count].funcs = adap_types[type].funcs;
			adapters[count].algo = adap_types[type].algo;
			count++;
		}
	}
	closedir(dir);

DONE:
	return adapters;
}

static int lookup_i2c_bus_by_name(const char *bus_name)
{
	struct i2c_adap *adapters;
	int i2cbus = -1;

	adapters = gather_i2c_busses();
	if (adapters == NULL) {
		fprintf(stderr, "Error: Out of memory!\n");
		return -3;
	}

	for (int i = 0; adapters[i].name; i++) {
		if (strcmp(adapters[i].name, bus_name) == 0) {
			if (i2cbus >= 0) {
				fprintf(stderr,
					"Error: I2C bus name is not unique!\n");
				i2cbus = -4;
				goto DONE;
			}
			i2cbus = adapters[i].nr;
		}
	}

	if (i2cbus == -1)
		fprintf(stderr, "Error: I2C bus name doesn't match any "
			"bus present!\n");

DONE:
	free_adapters(adapters);
	return i2cbus;
}

int lookup_i2c_bus(const char *i2cbus_arg)
{
	unsigned long i2cbus;
	char *end;

	i2cbus = strtoul(i2cbus_arg, &end, 0);
	if (*end || !*i2cbus_arg) {
		return lookup_i2c_bus_by_name(i2cbus_arg);
	}
	if (i2cbus > 0xFFFFF) {
		fprintf(stderr, "Error: I2C bus out of range!\n");
		return -2;
	}

	return (int)i2cbus;
}

int parse_i2c_address(const char *address_arg)
{
	long address;
	char *end;

	address = strtol(address_arg, &end, 0);
	if (*end || !*address_arg) {
		fprintf(stderr, "Error: Chip address is not a number!\n");
		return -1;
	}
	if (address < 0x03 || address > 0x77) {
		fprintf(stderr, "Error: Chip address out of range "
			"(0x03-0x77)!\n");
		return -2;
	}

	return (int)address;
}

int open_i2c_dev(int i2cbus, char *filename, size_t size, int quiet)
{
	int file;

	snprintf(filename, size, "/dev/i2c/%d", i2cbus);
	filename[size - 1] = '\0';
	file = open(filename, O_RDWR);

	if (file < 0 && (errno == ENOENT || errno == ENOTDIR)) {
		snprintf(filename, size, "/dev/i2c-%d", i2cbus);
		file = open(filename, O_RDWR);
	}

	if (file < 0 && !quiet) {
		if (errno == ENOENT) {
			fprintf(stderr, "Error: Could not open file "
				"`/dev/i2c-%d' or `/dev/i2c/%d': %s\n",
				i2cbus, i2cbus, strerror(ENOENT));
		} else {
			fprintf(stderr, "Error: Could not open file "
				"`%s': %s\n", filename, strerror(errno));
			if (errno == EACCES)
				fprintf(stderr, "Run as root?\n");
		}
	}

	return file;
}

int set_slave_addr(int file, int address, int force)
{
	if (ioctl(file, force ? I2C_SLAVE_FORCE : I2C_SLAVE, address) < 0) {
		fprintf(stderr,
			"Error: Could not set address to 0x%02x: %s\n",
			address, strerror(errno));
		return -errno;
	}

	return 0;
}
