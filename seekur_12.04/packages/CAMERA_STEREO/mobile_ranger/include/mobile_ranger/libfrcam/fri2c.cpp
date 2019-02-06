///////////////////////////////////////////////////////////////////////////
// Copyright (c) 2005 Focus Robotics. All rights reserved. 
//
// Created by    :  Jason Peck
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the 
// Free Software Foundation; either version 2 of the License, or (at your 
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but 
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
// General Public License for more details.
//
///////////////////////////////////////////////////////////////////////////
#include "frcam.h"
#include <stdio.h>
#include <errno.h>

struct i2c_cmd {
	unsigned int offset;
	unsigned int value;
	unsigned int devselect;
};

/* utility to read i2c csr */
int frI2CRead(int fd, unsigned int offset, unsigned int devselect) {
	struct i2c_cmd i2c;
	i2c.offset = offset;
	i2c.devselect = devselect;
	if (-1 == ioctl(fd, FR3_IOC_I2C_READ, &i2c)) {
		fprintf(stderr, "FR ERROR: FR3_IOC_I2C_READ failed for offset=%d, devselect=%d\n",
		       i2c.offset, i2c.devselect);
		return errno;
	}
	return i2c.value;
}

/* utility to write i2c csr */
int frI2CWrite(int fd, unsigned int offset, unsigned int value, unsigned int devselect) {
	struct i2c_cmd i2c;
	i2c.offset = offset;
	i2c.value = value;
	i2c.devselect = devselect;
	
	if (-1 == ioctl(fd, FR3_IOC_I2C_WRITE, &i2c)) {
		fprintf(stderr, "FR ERROR: FR3_IOC_I2C_WRITE failed for offset=%d, devselect=%d\n",
		       i2c.offset, i2c.devselect);
		return errno;
	}
	return 0;
}

/* utility to broadcast write i2c csr */
int frI2CWriteBC(int fd, unsigned int offset, unsigned int value) {
	struct i2c_cmd i2c;
	i2c.offset = offset;
	i2c.value = value;
	
	if (-1 == ioctl(fd, FR3_IOC_I2C_WRITE_BC, &i2c)) {
		fprintf(stderr, "FR ERROR: FR3_IOC_I2C_WRITE_BC failed for offset=%d\n",
		       i2c.offset);
		return errno;
	}
	return 0;
}
