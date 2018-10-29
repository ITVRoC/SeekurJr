/****************************************************************************
 * Copyright (c) 2006-2009 by Focus Robotics.
 *
 * All rights reserved. No part of this design may be reproduced stored
 * in a retrieval system, or transmitted, in any form or by any means,
 * electronic, mechanical, photocopying, recording, or otherwise, without
 * prior written permission of Focus Robotics, Inc.
 *
 * Proprietary and Confidential
 *
 * Created By   :  Andrew Worcester
 * Creation_Date:  Fri Dec 15 2006
 * 
 * Brief Description:
 * This class handles connections to and data transfer with an nDepth Processor
 * on an FR3 PCI or PC/104+ card (AKA Eagle).
 * 
 * Functionality:
 * 
 * Issues:
 * TODO: add interrupt support when the kernel driver supports it
 * TODO: Diagnostic messages should print via an FrMsgLog object
 *
 * Limitations:
 * 
 * Testing:
 * Needs to be tested with multiple eagle cards in one system.
 * 
 ******************************************************************************/
#include <stdio.h>  // for printf
#include <stdlib.h> // for exit
#include <fstream>  // for file io
#include <dirent.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h> // for mmap
#include <fcntl.h>    // for mmap
#include <cassert> // for assert

#include "FrEagleDevice.h" 
#define EAGLE_VENDOR_ID 0x105d
#define EAGLE_DEVICE_ID 0x850a


/*******************************************************************************/
/*******************************************************************************/
// CONSTRUCTORS/DESTRUCTOR

/**
 * Construtor which can be controlled from command line arguments. This is
 * primarily useful for small debug utilities where the setup needs to be 
 * controlled very precisely.
 */
FrEagleDevice::FrEagleDevice(FrOptArgs& args) { // Maybe pass in FrOptArgs class instead
  // Call any type of open and init based on command line arguments
  int camNum = args.getIntArg("camera-number");
  int doInit = args.getIntArg("init-camera");

  // If connection isn't specified, try eagle first, then sysfs or devmem
  if(openEagle(camNum)) {
    printf("FrEagleDevice: Fatal error trying to access hardware. Exiting...\n");
    exit(1);
  }
  if(doInit) {
    printf("FrEagleDevice: Initializing device\n");
    init(752, 480, 0);
  }
}

/**
 * Primary constructor. Tries to use the eagle kernel driver first, but falls
 * back to the sysfs connection if the eagle driver isn't available. DMA is
 * only available with the kernel driver.
 */
FrEagleDevice::FrEagleDevice(int id, int doInit) { // 0, 1, or 2 ints. Could default to -1 for next avail eagle dev
  // Try the eagle kernel driver first , fall back to access via sysfs
  int domain, bus, slot;
  if(openEagle(id)) {
    if((findPCIDev(EAGLE_VENDOR_ID, EAGLE_DEVICE_ID, &domain, &bus, &slot)!=1) ||
       openSysfs(domain, bus, slot)) {
      printf("Can't connect to Eagle Device. Aborting.\n");
      exit(1);
    }
  }
  if(doInit) init(752, 480, 0);
}

/**
 * Alternate constructor only allows access via mapping /dev/mem.
 * Limited use for debug purposes.
 */
FrEagleDevice::FrEagleDevice(int res0, int res1, int doInit) { // 3 ints
  if(openDevmem(res0, res1)) {
    printf("Can't connect to Eagle Device. Aborting.\n");
    exit(1);
  }
  if(doInit) init(752, 480, 0);
}

/**
 * Alternate constructor only allows access via sysfs.
 * Limited use for debug purposes.
 */
FrEagleDevice::FrEagleDevice(int domain, int bus, int slot, int doInit) { // 4 ints
  if(openSysfs(domain, bus, slot)) {
    printf("Can't connect to Eagle Device. Aborting.\n");
    exit(1);
  }
  if(doInit) init(752, 480, 0);
}


FrEagleDevice::~FrEagleDevice() {
  // Do any necessary cleanup
  // must close the device file, depending on what we opened to access eagle
  // FIXME: this is correct for openEagle, but not exactly right for openSysfs
  munmap(memBase, EAGLE_MEMORY_SPACE_LENGTH);
  munmap(csrBase, EAGLE_CSR_SPACE_LENGTH);
  close(devfile);  
}


/*******************************************************************************/
/*******************************************************************************/
// INITIALIZATION ROUTINES

/**
 * openEagle() connects to the eagle driver for access to the eagle hardware
 * id allows choosing which camera/processor to connect to, or -1 for the first
 * unused device.
 */
int FrEagleDevice::openEagle(int id) {
  char fname[63];
  sprintf(fname, "/dev/eagle%d", id);
  // Open dev and mmap resource regions for access to hardware
  // TODO: use id to make devfile name. Iterate until open succeeds for id=-1
  if((devfile=open(fname, O_RDWR))==-1) {
    printf("%s: ", fname);
    perror("Open failed for device file");
    return -1;
  }

  if((memBase=(int*)mmap(0, (EAGLE_MEMORY_SPACE_LENGTH), PROT_READ|PROT_WRITE, 
			 MAP_SHARED, devfile, EAGLE_MEMORY_SPACE_BASE))==MAP_FAILED) {
    printf("%s: ", fname);
    perror("mmap failed for memBase");
    return -1;
  }
  if((csrBase=(int*)mmap(0, (EAGLE_CSR_SPACE_LENGTH), PROT_READ|PROT_WRITE, 
			 MAP_SHARED, devfile, EAGLE_CSR_SPACE_BASE))==MAP_FAILED) {
    printf("%s: ", fname);
    perror("mmap failed for csrBase");
    return -1;
  }
  isOpen = true;
  noDMA = false;
  // Keep device file open for read() or ioctl() access
  return 0;
}

/**
 * openDevmem() allows access to Eagle by mmapping /dev/mem - in case there's no
 * sysfs available to us. This will probably rarely be used, but might be
 * helpful for certain debug or embedded machine situations.
 */
int FrEagleDevice::openDevmem(int res0, int res1) {
  // return immediately with error if uid isn't 0--only root user can access hw
  if(getuid()) {
    printf("Must be superuser to access Eagle camera via /dev/mem interface\n");
    // throw insufficient permissions exception
    return -1;
  }

  // standard mmap code here
  if((devfile=open("/dev/mem", O_RDWR))==-1) {
    perror("Open failed for /dev/mem");
    return -1;
  }

  if((memBase=(int*)mmap(0, (EAGLE_MEMORY_SPACE_LENGTH), PROT_READ|PROT_WRITE, 
			 MAP_SHARED, devfile, res1))==MAP_FAILED) {
    perror("mmap failed for /dev/mem memBase");
    return -1;
  }
  if((csrBase=(int*)mmap(0, (EAGLE_CSR_SPACE_LENGTH), PROT_READ|PROT_WRITE, 
			 MAP_SHARED, devfile, res0))==MAP_FAILED) {
    perror("mmap failed for /dev/mem csrBase");
    return -1;
  }

  isOpen = true;
  noDMA = true;
  return 0;
}


/**
 * openSysfs() directly accesses eagle hardware via the sysfs filesystem
 * The domain, bus, and slot numbers uniquely identify the Eagle PCI device in 
 * the system. It is always function 0 since an Eagle card isn't multifunction.
 * The process must have superuser privledges to call this function since the
 * resource files in /sys have limited access rights by default.
 */
int FrEagleDevice::openSysfs(int domain, int bus, int slot) {
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
  if((csrBase=(int*)mmap(0, (EAGLE_CSR_SPACE_LENGTH), PROT_READ|PROT_WRITE, 
			 MAP_SHARED, res0File, 0))==MAP_FAILED) {
    perror("mmap failed for csrBase");
    return -1;
  }

  if((res1File=open(res1Path, O_RDWR))==-1) {
    perror("Open failed for resource1 file");
    return -1;
  }
  if((memBase=(int*)mmap(0, (EAGLE_MEMORY_SPACE_LENGTH), PROT_READ|PROT_WRITE, 
			 MAP_SHARED, res1File, 0))==MAP_FAILED) {
    perror("mmap failed for memBase");
    return -1;
  }

  isOpen = true;
  noDMA = true;
  return 0;
}

/**
 *
 *
 */
int FrEagleDevice::findPCIDev(int vendor_id, int device_id, int *domain, int *bus, int *slot) { 
  // Scan through all PCI devices in /sys/bus/pci/devices
  printf("Opening /sys/bus/pci/devices to find devices with vendor_id %x and device_id %x.\n",
	 vendor_id, device_id);
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
 * init() support a full default initialization when specific control isn't needed
 * 
 */
int FrEagleDevice::init(int width, int height, int format) {

  if(initDMA()) { return -1; } // currently must be right after open

  // check that width is a multiple of four--transfer problems otherwise?
  assert (width%4 == 0);

  // Take the device out of reset
  setCSR(EAGLE_CSR_RESET, 0);

  // Set basic image properties
  setCSR(EAGLE_CSR_IMAGE_HEIGHT, height);
  setCSR(EAGLE_CSR_IMAGE_WIDTH, width);
  setCSR(EAGLE_CSR_IMAGE_FORMAT, format);

  if(initCalib()) { return -1; }

  return 0;
}


/**
 * initDMA() sets up the HW DMA engine and memory areas in host DMA memory
 * TODO: save these addresses in instance variables instead of assuming them 
 * again in the retreiveFrame() code below.
 * These addresses don't really have to always be set up this way, it's sort
 * of a waste of memory currently.
 */
int FrEagleDevice::initDMA() {
  if(noDMA) return 0;
  int bus_address;
  // Find the base bus address of the dma buffer
  // FIXME: hack! should get this via an IOCTL
  // FIXME: if read, lseek, or eventual ioctl fails, return error
  read(devfile, &bus_address, 4);
  //printf("Bus address for DMA buffer is %x\n", bus_address);
  lseek(devfile, 0, SEEK_SET);

  // set array of lseek offsets for the channel reads
  // set the dst addresses for each dma channel
  csrBase[EAGLE_CSR_PCIM_DST0] = bus_address;
  csrBase[EAGLE_CSR_PCIM_DST1] = bus_address + 0x100000;
  csrBase[EAGLE_CSR_PCIM_DST2] = bus_address + 0x200000;
  csrBase[EAGLE_CSR_PCIM_DST3] = bus_address + 0x300000;
  return 0;
}


/**
 * initCalib() does some housekeeping necessary to use the rectification core.
 * Basically, it programs coeff bursts and clears the remap table to identity
 * because random remap data can confuse the core sometimes.
 * I could also pass in a pointer to calib data, and only set to identity if
 * that pointer is null. That seems nice.
 */
int FrEagleDevice::initCalib() {
  int height = getCSR(EAGLE_CSR_IMAGE_HEIGHT);
  int width = getCSR(EAGLE_CSR_IMAGE_WIDTH);
  int widthTiles;
  if(width%128 == 0) { widthTiles = width/128; } 
  else {widthTiles = ( width/128) + 1; }
  // Note: this is only being computed for 16 pixel high calib tiles!
  // Stereo tiles are 8 pixels high and will be different then this
  int heightTiles;
  if(height%16 == 0) { heightTiles = height/16; } 
  else { heightTiles = (height/16) + 1; }
  int coeffBursts = (widthTiles * heightTiles) - 1;
  setCSR(EAGLE_CSR_CALIB_COEFF_BURSTS, coeffBursts);

  // FIXME: set calib remap memory to identity on startup

  return 0;
}


/*******************************************************************************/
/*******************************************************************************/
// LOW LEVEL DEVICE ACCESS

/**
 * setCSR() sets the csr at address offset to value data.
 * This function never fails. The only problem it could have would be if csrBase
 * was null or an unmapped value causing seq fault or bus error. Offset also 
 * can't be larger then 1MB or less then 0. In practical terms there's a problem
 * if offset is larger then 100.
 */
void FrEagleDevice::setCSR(int offset, int data) { 
  csrBase[offset] = data;
  msync(csrBase, (1024*1024), MS_SYNC); // force all mmap access to complete
}


/**
 * getCSR() returns the value of the csr at address offset
 * This function never fails. The only problem it could have would be if csrBase
 * was null.
 */
int FrEagleDevice::getCSR(int offset) const { 
  return csrBase[offset]; 
}

/**
 * waitCSR() waits for a particular CSR to change to a particular value.
 * Offset is the address of the csr we're waiting for
 * Value is the value we're waiting for it to become
 * Mask is set to 1 for any bits we wish to consider in the comparison
 * Timeout is how long in ms to wait for the csr to change to the correct value
 * The csr is polled once a milisecond and checked if it has the correct value
 * This function returns zero of the csr has the correct value, nonzero if the
 * check times out before the correct value is reached.
 */
int FrEagleDevice::waitCSR(int offset, int value, int mask, int timeout) const { 
  int timeCount;
  int found;
  assert(timeout > 0);
  assert(offset > 0);
  for ( timeCount = 0, found = 0; (timeCount < timeout) & !found; timeCount++ ) {
    if ( (mask&value) == (mask&getCSR(offset)) ) {
      found = 1;
    } else {
      usleep(1000);
    }
  } 
  return !found;
}

/**
 * Writes length dwords starting at offset dwords. Buffer data better be that
 * big! Length should be a multiple of 8 and start should be 8 dword aligned.
 */
void FrEagleDevice::writeMEM(int offset, int length, const int* data) { 
  assert (length > 0);
  assert ( (offset+length) <= 1024*1024*16 ); // Should check secondary subsystem enable for >16MB (>4MW)
  for (int i = 0; i < length; i++ ) {
    memBase[offset+i] = data[i];
  }
}

/**
 * Reads length dwords from Eagle memory starting at offset dwords. The buffer
 * pointed to by data better be big enough! Length should be a multiple of 8.
 * Transfers should be 8 dword aligned.
 */
void FrEagleDevice::readMEM(int offset, int length, int* data) const {  
  assert (length > 0);
  assert ( (offset+length) <= 1024*1024*16 );
  for (int i = 0; i < length; i++ ) {
    data[i] =  memBase[offset+i];
  }
}

/*******************************************************************************/
/*******************************************************************************/
// FRAME LEVEL ACCESS
//

/**
 * This function writes a 16 bit word data to device dev via the i2c bus. The 
 * hardware clock is 25KHz so it will normally take just about 1ms to complete
 * the transfer. Currently there is no automatic retry when an error is
 * detected. The function returns non-zero on error.
 */
int FrEagleDevice::writeWordI2C(int offset, int data, int dev) {
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_SLAVE_ID, dev);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_ADDR, offset);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_RW, 0);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_DATA, data);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 1);
  if(waitCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 0x0, 0x1, 10)) {
    printf("EagleDevice: I2C operation timeout when writing to device %x\n", dev);
    printf("EagleDevice: offset is %d, error flag is %x\n", offset, 
	   getCSR(EAGLE_CSR_I2C_DEVICE_CSR_ERROR));
    setCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 0);
    return 1;
  }
  return 0;
}


/**
 * This function reads a 16 bit word data from device dev via the i2c bus. The 
 * hardware clock is 25KHz so it will normally take just about 1ms to complete
 * the transfer. Currently there is no automatic retry when an error is
 * detected. The function returns non-zero on error.
 */
int FrEagleDevice::readWordI2C(int offset, int *data, int dev) {
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_SLAVE_ID, dev);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_ADDR, offset);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_RW, 1);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 1);
  if(waitCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 0x0, 0x1, 10)) {
    printf("EagleDevice: I2C operation timeout when writing to device %x\n", dev);
    printf("EagleDevice: offset is %d, error flag is %x\n", offset, 
	   getCSR(EAGLE_CSR_I2C_DEVICE_CSR_ERROR));
    setCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 0);
    return 1;
  }
  *data = getCSR(EAGLE_CSR_I2C_DEVICE_CSR_DATA);
  return 0;
}


/**
 * setI2CCSR() (change to writeI2C())
 * This camera is specific to the Eagle stereo camera and should be moved to that class
 */
void FrEagleDevice::setI2CCSR(int offset, int value, int devSelect) {
  
  if ( devSelect == -1 ) {
    // bcast
    setCSR(EAGLE_CSR_I2C_DEVICE_CSR_BCAST, 1);
    // slave and master share same B8 address
    setCSR(EAGLE_CSR_I2C_DEVICE_CSR_SLAVE_ID, 0xB8); 
  } else {
    // pt2pt
    setCSR(EAGLE_CSR_I2C_DEVICE_CSR_BCAST, 0);
    
    if ( devSelect ) {
      // slave is B8 for a write when bcast=0
      setCSR(EAGLE_CSR_I2C_DEVICE_CSR_SLAVE_ID, 0xB8);     
    } else {
      // master is B0 for a write when bcast=0
      setCSR(EAGLE_CSR_I2C_DEVICE_CSR_SLAVE_ID, 0xB0); 
    }
  }
      
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_ADDR, offset);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_RW, 0);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_DATA, value);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 1);
  
  if(waitCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 0, 1, 10)) {
  //while ( getCSR(EAGLE_CSR_I2C_DEVICE_CSR_START) == 1 ) {
    // FIXME: This should probably be an exception.
    if ( getCSR(EAGLE_CSR_I2C_DEVICE_CSR_ERROR) == 1 ) {
      fprintf( stderr, "setI2CCSR faild for offset=%x, value=%x", offset, value);
      printf("Error bit set. Clearing error by clearing csr_start\n");
      setCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 0);
    } else {
      fprintf( stderr, "setI2CCSR faild for offset=%x, value=%x", offset, value);
      printf("Transaction timeout with no error bit set. Clearing csr_start\n");
      setCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 0);	
    }
  }
  
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_BCAST, 0);        

  // Persistant error is unlikely and indicates I2C FSM problem
    if ( getCSR(EAGLE_CSR_I2C_DEVICE_CSR_ERROR) == 1 ) {
    // FIXME: This should probably be an exception.
    fprintf( stderr, "setI2CCSR faild for offset=%x, value=%x", offset, value);
  }

  msync(csrBase, (1024*1024), MS_SYNC); // force all mmap access to complete
}

/**
 * getI2CCSR() (change to readI2C())
 * This function is specific to the Eagle stereo camera and should be moved to that class
 */
int FrEagleDevice::getI2CCSR(int offset, int devSelect) {
  
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_BCAST, 0); 

  if ( devSelect ) // slave is B9 for read when bcast=0
    setCSR(EAGLE_CSR_I2C_DEVICE_CSR_SLAVE_ID, 0xB8); 
  else          // master is B1 for read when bcast=0
    setCSR(EAGLE_CSR_I2C_DEVICE_CSR_SLAVE_ID, 0xB0); 

  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_ADDR, offset);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_RW, 1);
  setCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 1);
  
  // FIXME: FIXME: FIXME: FIXME:
  // The I2C hardware is currently somewhat broken because when the AEGC circuit
  // is running we can lose read data. Thus this function must poll in a tight 
  // loop instead of once per ms. This substantially increases CPU overhead, but
  // it must remain this way until I rewrite the I2C master code and AEGC code.
  //if(waitCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 0, 1, 10)) {
  while ( getCSR(EAGLE_CSR_I2C_DEVICE_CSR_START) == 1 ) {
    if ( getCSR(EAGLE_CSR_I2C_DEVICE_CSR_ERROR) == 1 ) {
      // FIXME: This should probably be an exception.
      fprintf( stderr, "getI2CCSR faild for offset=%x\n", offset);
      printf("Clearing error by clearing csr_start\n");
      setCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 0);
    } else {
      //fprintf( stderr, "getI2CCSR faild for offset=%x", offset);
      //printf("Transaction timeout with no error bit set. Clearing csr_start\n");
      //setCSR(EAGLE_CSR_I2C_DEVICE_CSR_START, 0);
    }
  }

  // Persistant error is unlikely and indicates I2C FSM problem
  if ( getCSR(EAGLE_CSR_I2C_DEVICE_CSR_ERROR) == 1) {
    // FIXME: This should probably be an exception.
    fprintf( stderr, "getI2CCSR faild for offset=%x\n", offset);
  };

  return getCSR(EAGLE_CSR_I2C_DEVICE_CSR_DATA);
}


/*******************************************************************************/
/*******************************************************************************/
// FRAME LEVEL ACCESS
//

/**
 * grabFrame() initiates a DMA operation and returns
 * Use retrieveFrame() below to actually get the data transferred from the DMA
 * chan specifies which of Eagle's 4 DMA channels to use
 * src specifies which frame in memory to grab
 * If grabFrame is called again when that channel is already active, it does
 * nothing-always interleave grab's and receives to a channel to avoid this case
 * 
 * NOTE: the HW DMA engine uses the width and height set in CSR's to determine
 * the actual data size to transfer. Stride will be width padded out to be a
 * multiple of 32 bytes if it isn't already one.
 * NOTE: DMA DST addresses _must_ be set properly before calling this. Currently
 * they are set up in init().
 */
void FrEagleDevice::grabFrame(int chan, int src) {
  if(noDMA) { 
    if(chan<4 && chan>=0) chanSrc[chan] = src;
    else printf("FrEagleDevice::grabFrame: illegal channel requested %d\n", chan);
    return; 
  }
  int status = csrBase[EAGLE_CSR_DMAC_STATUS];
  //printf("FrEagleDevice::grabFrame: chan=%d src=%d status=%x\n", chan, src, status);
  switch(chan) {
  case 0:
    if((status & 0x100) == 0) {
      csrBase[EAGLE_CSR_DMAC_STATUS] = 0;
      //printf("Starting dma on chan 0\n");
      csrBase[EAGLE_CSR_DMAC_SRC0]=src;
    }
    break;
  case 1:
    if((status & 0x200) == 0) {
      csrBase[EAGLE_CSR_DMAC_STATUS] = 0;
      //printf("Starting dma on chan 1\n");
      csrBase[EAGLE_CSR_DMAC_SRC1]=src;
    }
    break;
  case 2:
    if((status & 0x400) == 0) {
      csrBase[EAGLE_CSR_DMAC_STATUS] = 0;
      //printf("Starting dma on chan 2\n");
      csrBase[EAGLE_CSR_DMAC_SRC2]=src;
    }
    break;
  case 3:
    if((status & 0x800) == 0) {
      csrBase[EAGLE_CSR_DMAC_STATUS] = 0;
      //printf("Starting dma on chan 3\n");
      csrBase[EAGLE_CSR_DMAC_SRC3]=src;
    }
    break;
  default:
    printf("FrEagleDevice::grabFrame: illegal channel requested %d\n", chan);
    return;
  }
}


/**
 * retrieveFrame() actually returns images that have been DMA'd from the hardware
 * It's only usefull when called following a call to grabFrame(), above.
 * chan is the chanel that was grabbed with grabFrame()
 *
 * FIXME: don't used the fixed size read that assumes 752x480--should I pass read
 *        size in?
 * FIXME: don't assume read offset for each channel. Save the addresses that this
 *        was set to in initDMA() above and use those numbers.
 * MAYBE: should I be able to select row stride like I can in readFrame()?
 * INFO: right now the returned data will always have a stride of 768, the hw dma width
 */
int FrEagleDevice::retrieveFrame(int chan, uchar* data) {
  if(noDMA) {
    if(chan<4 && chan>=0) readFrame(chanSrc[chan], data, 768);
    else printf("FrEagleDevice::retrieveFrame: illegal channel requested %d\n", chan);
    return 0;
  }

  switch(chan) {
  case 0:
    if(waitCSR(EAGLE_CSR_DMAC_STATUS, 0x000, 0x100, 50)) {
      printf("FrEagleDevice::retrieveFrame chan=%d: timeout waiting for dma completion\n", chan);
      return -1;
    }
    if(lseek(devfile, 0x000000, SEEK_SET)==-1) { perror("FrEagleDevice chan0 lseek:"); }
    if(read(devfile, data, 368640)==-1) { perror("FrEagleDevice chan0 read:"); }
    return 0;
  case 1:
    if(waitCSR(EAGLE_CSR_DMAC_STATUS, 0x000, 0x200, 50)) {
      printf("FrEagleDevice::retrieveFrame chan=%d: timeout waiting for dma completion\n", chan);
      return -1;
    }
    if(lseek(devfile, 0x100000, SEEK_SET)==-1) { perror("FrEagleDevice chan1 lseek:"); }
    if(read(devfile, data, 368640)==-1) { perror("FrEagleDevice chan1 read:"); }
    return 0;
  case 2:
    if(waitCSR(EAGLE_CSR_DMAC_STATUS, 0x000, 0x400, 50)) {
      printf("FrEagleDevice::retrieveFrame chan=%d: timeout waiting for dma completion\n", chan);
      return -1;
    }
    if(lseek(devfile, 0x200000, SEEK_SET)==-1) { perror("FrEagleDevice chan2 lseek:"); }
    if(read(devfile, data, 368640)==-1) { perror("FrEagleDevice chan2 read:"); }
    return 0;
  case 3:
    if(waitCSR(EAGLE_CSR_DMAC_STATUS, 0x000, 0x800, 50)) {
      printf("FrEagleDevice::retrieveFrame chan=%d: timeout waiting for dma completion\n", chan);
      return -1;
    }
    if(lseek(devfile, 0x300000, SEEK_SET)==-1) { perror("FrEagleDevice chan3 lseek:"); }
    if(read(devfile, data, 368640)==-1) { perror("FrEagleDevice chan3 read:"); }
    return 0;
  default:
    printf("FrEagleDevice::retrieveFrame: illegal channel requested %d\n", chan);
    return -1;
  }
}


/**
 * readFrame() reads a single image frame from the Eagle device.
 * The offset is the image number in memory--a 1MB aligned location.
 * The data pointer must point to a large enough chunk of free memory to hold 
 * the entire image.
 * Step is the row stride to use for data, in bytes.
 *
 * This code assumes the hardware frame starts on aligned 1MB boundaries and
 * that the stride in memory is 1KB.
 *
 * Should frame width and height be added to this function definition?
 * Is it reasonable to assume that offset is in megabytes? It's OK for now since hw does
 * Maybe set this up as width and padding instead of full, sub, nop
 */
void FrEagleDevice::readFrame(int offset, uchar* data, int step) const {
  assert ( offset < 32 );
  assert ( step%4 == 0 );

  int frameBase  = offset * 1024 * 1024 / 4;
  int height = getCSR(EAGLE_CSR_IMAGE_HEIGHT);
  int width = getCSR(EAGLE_CSR_IMAGE_WIDTH);
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
      for( xEntry = 0; xEntry < 8; xEntry++ ) { // should we unroll this loop?
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
 * writeFrame() writes a single image frame into the Eagle device.
 * The offset is the MB aligned location in the HW to write the image
 * The data is a pointer to the bytes to write into the HW
 * The step is the distance between successive lines in bytes in the data array
 *
 * This code assumes the hardware frame starts on aligned 1MB boundaries and
 * that the stride in memory is 1KB.
 *
 * Should frame width and height be added to this function definition?
 * Is it reasonable to assume that offset is in megabytes? It's OK for now since hw does
 * Maybe set this up as width and padding instead of full, sub, nop
 */
void FrEagleDevice::writeFrame(int offset, uchar* data, int step) {
  assert ( offset < 32 );  
  assert ( step%4 == 0 );

  int frameBase  = offset * 1024 * 1024 / 4;
  int height = getCSR(EAGLE_CSR_IMAGE_HEIGHT);
  int width = getCSR(EAGLE_CSR_IMAGE_WIDTH);
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
      for( xEntry = 0; xEntry < 8; xEntry++ ) { // should we unroll this loop?
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
 * Just a quick, simple way to clear out (or set) any buffer. Useful for debug.
 * Clears the whole 1MB, no need for a frame size spec.
 */
void FrEagleDevice::clearFrame(int offset, const uchar val) {
  int frameBase = offset * 1024 * 1024 / 4;
  for(int y=0; y<1023; y++) {
    for(int x=0; x<256; x++) {
      memBase[frameBase++] = val;
    }
  }
}

/**
 *
 *
 */
//void FrEagleDevice::writeRectMap() {
//
//}
/*******************************************************************************/
/*******************************************************************************/
// UTILITY ROUTINES

/**
 *
 */
// Return offset of a names csr, or negative if unrecognized
// Name may be the EAGLE_CSR_* name, or the numerical offset in string form
int FrEagleDevice::str2csr(const char* csrName) {
  // first, analyze the string to determine if it's a csr name or a number
  // it's a name if it starts with EAGLE_CSR_ (any case) and it's a number if
  // it starts with 0x and then is all numbers or if it starts with 0 and is all
  // numbers, or if it's all numeric. If it's neither a csr name or a number 
  // then how do I report the error? Probably by returning a negative value
  // which can never be a valid csr, by convention.

  // So, since I want it to be OK to have the offset in parens at the end of
  // the csr name string, I will only consider the string up to the first space.
  // So a valid name must start with csr_addr_ (any case) and is compared to 
  // known good values only up to the first space. Otherwise it fails.

  // The input string will thus first be analyzed and if it's a hex, octal, or
  // decimal numeric string it will be converted and the function return. If it's
  // a valid alpha string then it is copied up to the first space into a tmp
  // string and converted to all upper case and compared against known csr names
  // in a huge if-elseif-elseif statement. If it fails any check or drops 
  // through the elseif block, then -1 is returned.

  // FIXME: quick hack to get started.
  int val = strtol(csrName, 0, 0);
  printf("CSR Name is %s which converts to %d(0x%x)\n", csrName, val, val);
  return val;
}

/**
 *
 */
// Somehow, it seems like this should be automatically generated from the csr defines list
const char* FrEagleDevice::csr2str(int csr) {
  // this function is much easier. Just a straight case statement on the int
  // input value. The default, if no matches, is "INVALID_CSR_NAME (-1)"
  // It would be nice if all csr names get their offset appended to them, but it
  // may be necessary to turn that off, so maybe a control param needs to be 
  // added. The only real reason to turn it off would be that str2csr wouldn't
  // understand it, so maybe better to just make str2csr work fine with the 
  // offset added in parens.
  switch(csr) {
  case 0: return "EAGLE_CSR_CHIP_VERSION";
  case 1: return "EAGLE_CSR_IMAGE_HEIGHT";
  case 2: return "EAGLE_CSR_IMAGE_WIDTH";
  case 3: return "EAGLE_CSR_IMAGE_FORMAT";
  case 4: return "EAGLE_CSR_VRX_BASE_ADDR_CH0";
  case 5: return "EAGLE_CSR_VRX_BASE_ADDR_CH1";
  case 6: return "EAGLE_CSR_VRX_FRAME_CNT";
  case 7: return "EAGLE_CSR_VRX_FRAME_ERROR";
  case 8: return "EAGLE_CSR_VRX_LINE_CNT";
  case 9: return "EAGLE_CSR_VRX_FRAME_SKIP_N";
  case 10: return "EAGLE_CSR_VRX_FRAME_SKIP_M";
  case 11: return "EAGLE_CSR_VRX_EN";
  case 12: return "EAGLE_CSR_VRX_RPWDN";
  case 13: return "EAGLE_CSR_VRX_LOCK";
  case 14: return "EAGLE_CSR_VRX_ERROR";
  case 15: return "EAGLE_CSR_VRX_ERROR_STATUS";
  case 16: return "EAGLE_CSR_I2C_DEVICE_CSR_START";
  case 17: return "EAGLE_CSR_I2C_DEVICE_CSR_SLAVE_ID";
  case 18: return "EAGLE_CSR_I2C_DEVICE_CSR_ADDR";
  case 19: return "EAGLE_CSR_I2C_DEVICE_CSR_RW";
  case 20: return "EAGLE_CSR_I2C_DEVICE_CSR_DATA";
  case 21: return "EAGLE_CSR_I2C_DEVICE_CSR_ERROR";
  case 22: return "EAGLE_CSR_I2C_DEVICE_CSR_BCAST";
  case 23: return "EAGLE_CSR_I2C_DEVICE_CSR_DEVSEL";
  case 24: return "EAGLE_CSR_CALIB_ENABLE";
  case 25: return "EAGLE_CSR_CALIB_STATUS";
  case 26: return "EAGLE_CSR_CALIB_LEFT_SRC";
  case 27: return "EAGLE_CSR_CALIB_RIGHT_SRC";
  case 28: return "EAGLE_CSR_CALIB_LEFT_DST";
  case 29: return "EAGLE_CSR_CALIB_RIGHT_DST";
  case 30: return "EAGLE_CSR_STEREO_ENABLE";
  case 31: return "EAGLE_CSR_STEREO_STATUS";
  case 32: return "EAGLE_CSR_STEREO_LEFT_SRC";
  case 33: return "EAGLE_CSR_STEREO_RIGHT_SRC";
  case 34: return "EAGLE_CSR_STEREO_DISP_DST";
  case 35: return "EAGLE_CSR_STEREO_SEARCH_MODE";
  case 36: return "EAGLE_CSR_STEREO_SEARCH_RANGE";
  case 37: return "EAGLE_CSR_DMAC_SRC0";
  case 38: return "EAGLE_CSR_DMAC_SRC1";
  case 39: return "EAGLE_CSR_DMAC_SRC2";
  case 40: return "EAGLE_CSR_DMAC_SRC3";
  case 41: return "EAGLE_CSR_DMAC_STATUS";
  case 42: return "EAGLE_CSR_PCIM_DST0";
  case 43: return "EAGLE_CSR_PCIM_DST1";
  case 44: return "EAGLE_CSR_PCIM_DST2";
  case 45: return "EAGLE_CSR_PCIM_DST3";
  case 46: return "EAGLE_CSR_SDRAM1_OWNER_SELECT";
  case 47: return "EAGLE_CSR_DCM_STATUS";
  case 48: return "EAGLE_CSR_INT_STAT";
  case 49: return "EAGLE_CSR_INT_MASK";
  case 50: return "EAGLE_CSR_GPIO";
  case 51: return "EAGLE_CSR_RESET";
  case 52: return "EAGLE_CSR_VRX_STATUS";
  case 53: return "EAGLE_CSR_CALIB_COEFF_BURSTS";
  case 54: return "EAGLE_CSR_AEGC_ENABLE";
  case 55: return "EAGLE_CSR_AEGC_SKIP_FRAMES";
  case 56: return "EAGLE_CSR_DEBUG_CTL";
  case 57: return "EAGLE_CSR_DEBUG_DAT0";
  case 58: return "EAGLE_CSR_VTX_CTL";
  default: return "INVALID_CSR_NAME";
  }
}



