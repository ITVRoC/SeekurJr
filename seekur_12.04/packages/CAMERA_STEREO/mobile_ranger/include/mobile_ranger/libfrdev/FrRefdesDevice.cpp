/****************************************************************************
 * Copyright (c) 2007 by Focus Robotics.
 *
 * All rights reserved. No part of this design may be reproduced stored
 * in a retrieval system, or transmitted, in any form or by any means,
 * electronic, mechanical, photocopying, recording, or otherwise, without
 * prior written permission of Focus Robotics, Inc.
 *
 * Proprietary and Confidential
 *
 * Created By   :  Andrew Worcester
 * Creation_Date:  Wed Mar 28 2007
 * 
 * Brief Description:
 * This class connects to and handles accesses to the reference design for the
 * Focus Robotics nDepth Development System.
 * 
 * Functionality:
 * 
 * Issues:
 * 
 * Limitations:
 * 
 * Testing:
 * 
 ******************************************************************************/
#include <stdio.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <cassert>
#include "FrRefdesDevice.h"

#define REFDES_VENDOR_ID 0x105d
#define REFDES_DEVICE_ID 0x850b

// CONSTRUCTOR/DESTRUCTOR
FrRefdesDevice::FrRefdesDevice(int id, int doInit) {
  int domain, bus, slot;
  findPCIDev(REFDES_VENDOR_ID, REFDES_DEVICE_ID, &domain, &bus, &slot);
  openSysfs(domain, bus, slot);
  setCSR(REFDES_CSR_RESET, 0);
}

FrRefdesDevice::~FrRefdesDevice() {}


// INIT FUNCTIONS

/**
 *
 */
int FrRefdesDevice::findPCIDev(int vendor_id, int device_id, int *domain, int *bus, int *slot) { 
  // Scan through all PCI devices in /sys/bus/pci/devices
  printf("Opening /sys/bus/pci/devices to find matching devices.\n");
  DIR *pcidevs = opendir("/sys/bus/pci/devices");
  if(pcidevs==NULL) perror("Couldn't open pci devices directory");

  // For each device, open the vendor file and check if it's contents match
  // the vendor_id and device_id provided to this function
  int num_match = 0;
  struct dirent *devdir;
  char idfilepath[64];
  FILE *idfile;
  int id;
  while((devdir=readdir(pcidevs))) {
    if(devdir->d_name[0]=='.') continue; // skip . and .. entries
    printf("Scanning %s... ", devdir->d_name);

    // Check if pci vendor id is correct for this device
    sprintf(idfilepath, "/sys/bus/pci/devices/%s/vendor", devdir->d_name);
    idfile = fopen(idfilepath, "r");
    if(!idfile) perror("Open failed:");
    fscanf(idfile, "%x", &id);
    fclose(idfile);
    printf("found vendor ID of %x ", id);

    // If vendor id is correct, then check whether device id matches
    if(id==vendor_id) {
      sprintf(idfilepath, "/sys/bus/pci/devices/%s/device", devdir->d_name);
      idfile = fopen(idfilepath, "r");
      if(!idfile) perror("Open failed:");
      fscanf(idfile, "%x", &id);
      fclose(idfile);
      printf("found device ID of %x ", id);

      // If both vendor and device match, then save the path and increment the
      // number of matches that we've found
      if(id==device_id) {
	//sprintf(dirpath, "/sys/bus/pci/devices/%s", devdir->d_name);
	sscanf(devdir->d_name, "%4x:%2x:%2x.0", domain, bus, slot);
	num_match++;
	printf("...Exact match! Device is %4.4x:%2.2x:%2.2x.0", *domain, *bus, *slot);
      }
    }
    printf("\n");
  }

  // return number of matching devices found
  return num_match;
}

/**
 * openSysfs() directly accesses eagle hardware via the sysfs filesystem
 * The domain, bus, and slot numbers uniquely identify the Eagle PCI device in 
 * the system. It is always function 0 since an Eagle card isn't multifunction.
 * The process must have superuser privledges to call this function since the
 * resource files in /sys have limited access rights by default.
 */
int FrRefdesDevice::openSysfs(int domain, int bus, int slot) { 
  // return immediately with error if uid isn't 0--only root user can access hw
  if(getuid()) {
    printf("Must be superuser to access Eagle camera via sysfs interface\n");
    // throw insufficient permissions exception
    return -1;
  }

  // Create the paths and filename for device files to mmap
  char sysfsPath[63];
  char res0Path [63];
  char res1Path [63];
  sprintf(sysfsPath, "/sys/bus/pci/devices/%4.4x:%2.2x:%2.2x.0", domain, bus, slot);
  sprintf(res0Path, "%s/resource0", sysfsPath);
  sprintf(res1Path, "%s/resource1", sysfsPath);

  // Mmap the appropriate files to get direct access to PCI memory regions
  int res0File, res1File;
  if((res0File=open(res0Path, O_RDWR))==-1) {
    perror("Open failed for resource0 file");
    return -1;
  }
  if((csrBase=(int*)mmap(0, (REFDES_CSR_SPACE_LENGTH), PROT_READ|PROT_WRITE, 
			 MAP_SHARED, res0File, 0))==MAP_FAILED) {
    perror("mmap failed for csrBase");
    return -1;
  }

  if((res1File=open(res1Path, O_RDWR))==-1) {
    perror("Open failed for resource1 file");
    return -1;
  }
  if((memBase=(int*)mmap(0, (REFDES_MEMORY_SPACE_LENGTH), PROT_READ|PROT_WRITE, 
			 MAP_SHARED, res1File, 0))==MAP_FAILED) {
    perror("mmap failed for memBase");
    return -1;
  }

  return 0;
}


// LOW LEVEL FUNCTIONS
/**
 * Sets the value of the CSR at location offset to data
 */
void FrRefdesDevice::setCSR(int offset, int data) {
  csrBase[offset] = data;
}

/**
 * Returns the value of the CSR read at location offset
 */
int FrRefdesDevice::getCSR(int offset) const { 
  return csrBase[offset]; 
}

/**
 * Waits up to timeout miliseconds for the CSR at offset to match the value in
 * data in all bit positions except where mask is set to zero. Returns non-zero
 * if the time passes before the CSR changes to the desired value.
 */
int FrRefdesDevice::waitCSR(int offset, int data, int mask, int timeout) const { 
  int timeCount;
  int found;
  assert(timeout > 0);
  assert(offset > 0);
  for ( timeCount = 0, found = 0; (timeCount < timeout) & !found; timeCount++ ) {
    if ( (mask&data) == (mask&getCSR(offset)) ) {
      found = 1;
    } else {
      usleep(1000);
    }
  } 
  return !found;
}

/**
 * Transfers length dwords of data from the buffer pointed to by data on the
 * host to the SDRAM location pointed to by offset on the PCI card.
 * Transfers should always start at 32-byte aligned locations and be a mult
 * of 8 dwords in length.
 */
void FrRefdesDevice::writeMEM(int offset, int length, const int* data) {
  assert (length > 0);
  assert ( (offset+length) <= 1024*1024*16 ); // Should check secondary subsystem enable for >16MB (>4MW)
  for (int i = 0; i < length; i++ ) {
    memBase[offset+i] = data[i];
  }
}

/**
 * Transfers length dwords from SDRAM on the PCI card to the buffer pointed to
 * by data on the host. Transfers start offset dwords from the start of SDRAM on
 * the PCI card.
 * Transfers should always start at 32-byte aligned locations and be a mult
 * of 8 dwords in length.
 */
void FrRefdesDevice::readMEM(int offset, int length, int* data) const {
  assert (length > 0);
  assert ( (offset+length) <= 1024*1024*16 );
  for (int i = 0; i < length; i++ ) {
    data[i] =  memBase[offset+i];
  }
}


// HIGHER LEVEL FUNCTIONS
/**
 * readFrame() reads a single image frame from the Refdes device.
 * The offset is the image number in memory--a 1MB aligned location.
 * The data pointer must point to a large enough chunk of free memory to hold 
 * the entire image.
 * Step is the row stride to use for data, in bytes.
 *
 * This code assumes the hardware frame starts on aligned 1MB boundaries and
 * that the stride in memory is 1KB. This code further assumes that the image
 * to be transferred with be 752x480 pixels.
 */
void FrRefdesDevice::readFrame(int offset, char* data, int step) const {
  assert ( offset < 32 );
  assert ( step%4 == 0 );

  int frameBase  = offset * 1024 * 1024 / 4;
  int height = 480;
  int width = 752;
  int fullBursts = width/32;
  int subBursts = (width - fullBursts*32) / 4;

  int yStepImg;
  int yStepMem;

  int xBurst;
  int xEntry;
  int xBurstStep;

  volatile int nop;

  for (int y = 0; y < height; y++ ) {
    yStepImg = y * (step/4);
    yStepMem = frameBase + ( y * 256 ); // width step in hardware is always 256 dwords

    // full bursts
    for( xBurst = 0; xBurst < fullBursts; xBurst++ ) {
      xBurstStep = xBurst * 8;
      for( xEntry = 0; xEntry < 8; xEntry++ ) {
	((int*)data + yStepImg)[xBurstStep+xEntry] = memBase[yStepMem+xBurstStep+xEntry];
      }
    }
    //sub bursts
    xBurstStep = xBurst * 8;
    for( xEntry = 0; xEntry < subBursts; xEntry++ ) {
      ((int*)data + yStepImg)[xBurstStep+xEntry] = memBase[yStepMem+xBurstStep+xEntry];
    }
    // nops
    for (int nopEntry = 8 - xEntry; nopEntry < 8; nopEntry++ ) {
      nop = memBase[yStepMem+xBurstStep+nopEntry]; 
    }
  } 

}

/**
 * Loads an image into refdes SDRAM from a character buffer provided. Stride is
 * adjusted from whatever is used on the host to the assumed 1KB stride in the
 * hardware. Operations are padded to always write in increments of 32 bytes.
 * Data is written to frame number offset.
 *
 * writeFrame() writes a single image frame into the Refdes device.
 * The offset is the MB aligned location in the HW to write the image
 * The data is a pointer to the bytes to write into the HW
 * The step is the distance between successive lines in bytes in the data array
 *
 * This code assumes the hardware frame starts on aligned 1MB boundaries and
 * that the stride in memory is 1KB. It further assumes the image will be 752
 * by 480 pixels.
 */
void FrRefdesDevice::writeFrame(int offset, const char* data, int step) {
  assert ( offset < 32 );  
  assert ( step%4 == 0 );

  int frameBase  = offset * 1024 * 1024 / 4;
  int height = 480;
  int width = 752;
  int fullBursts = width/32;
  int subBursts = (width - fullBursts*32) / 4;

  int yStepImg;
  int yStepMem;

  int xBurst;
  int xEntry;
  int xBurstStep;

  for (int y = 0; y < height; y++ ) {
    yStepImg = y * (step/4); // width step in the src image converted to dwords
    yStepMem = frameBase + ( y * 256 ); // width step in hardware is always 256 dwords

    // full bursts
    for( xBurst = 0; xBurst < fullBursts; xBurst++ ) {
      xBurstStep = xBurst * 8;
      for( xEntry = 0; xEntry < 8; xEntry++ ) {
	memBase[yStepMem+xBurstStep+xEntry] = ((int*)data + yStepImg)[xBurstStep+xEntry];
      }
    }
    //sub bursts
    xBurstStep = xBurst * 8;
    for( xEntry = 0; xEntry < subBursts; xEntry++ ) {
      memBase[yStepMem+xBurstStep+xEntry] = ((int*)data + yStepImg)[xBurstStep+xEntry];
    }
    // nops
    for (int nopEntry = 8 - xEntry; nopEntry < 8; nopEntry++ ) {
      memBase[yStepMem+xBurstStep+nopEntry] = 0;
    }
  } 
}

/**
 * Clears an entire 1MB frame to a constant value (by default 0) in the SDRAM 
 * on the refdes board. Which 1MB frame is selected by the value of offset.
 * Every byte in the frame is set to the value data.
 */
void FrRefdesDevice::clearFrame(int offset, const char data) {
  int frameBase = offset * 1024 * 1024 / 4;
  int dword = data | (data<<8) | (data<<16) | (data<<24);
  for(int y=0; y<1023; y++) {
    for(int x=0; x<256; x++) {
      memBase[frameBase++] = dword;
    }
  }
}

/**
 * Writes the two low order bytes in data to device dev on the I2C bus at offset
 * offset. Returns non-zero if any errors were encountered during the operation.
 */
int FrRefdesDevice::writeWordI2C(int offset, int data, int dev) { 
return 0; 
}

/**
 * Reads two bytes into the low order bytes of data from device dev on the I2C
 * bus at offset offset. Returns non-zero if any errors were encountered during
 * the operation.
 */
int FrRefdesDevice::readWordI2C(int offset, int *data, int dev) { 
return 0; 
}


