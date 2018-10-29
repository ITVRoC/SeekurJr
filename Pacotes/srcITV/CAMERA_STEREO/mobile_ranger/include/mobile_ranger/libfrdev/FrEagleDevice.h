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
 * 
 * Functionality:
 * 
 * Issues:
 * TODO: add exception classes and properly throw exceptions as required
 *
 * Limitations:
 * 
 * Testing:
 * 
 ******************************************************************************/
#ifndef FREAGLEDEVICE_H
#define FREAGLEDEVICE_H

#include "FrOptArgs.h"
#include "FrEagleDeviceRegs.h"
typedef unsigned char uchar;

/// Provides access to Eagle PCI and PC/104+ hardware.
/**
 * This class handles connecting to an Eagle device and manages the connection
 * once made. It provides register and memory access, as well as some functions
 * to transfer full video frames either via mmap or DMA. It provides access to 
 * the I2C bus via Eagle.
 */
class FrEagleDevice {
 public:
  // CONSTRUCTORS/DESTRUCTORS
  FrEagleDevice(FrOptArgs& args);
  FrEagleDevice(int id=0, int doInit=1);
  FrEagleDevice(int res0, int res1, int doInit);
  FrEagleDevice(int domain, int bus, int slot, int doInit);
  ~FrEagleDevice();

  // INIT FUNCTIONS
  int openEagle(int id);
  int openDevmem(int res0, int res1);
  int openSysfs(int domain, int bus, int slot);
  int findPCIDev(int vendor_id, int device_id, int *domain, int *bus, int *slot);
  int init(int width, int height, int format);
  int initDMA();
  int initCalib();

  // LOW LEVEL FUNCTIONS
  void setCSR(int offset, int data);
  int getCSR(int offset) const;
  int waitCSR(int offset, int value, int mask=-1, int timeout=50) const;
  void writeMEM(int offset, int length, const int* data);
  void readMEM(int offset, int length, int* data) const;  

  // I2C ACCESS FUNCTIONS
  int writeWordI2C(int offset, int data, int dev);
  int readWordI2C(int offset, int *data, int dev);
  void setCameraCSR(int offset, int value, int camSel=-1);
  int getCameraCSR(int offset, int camSel);
  void setI2CCSR(int offset, int value, int devSelect = -1);
  int getI2CCSR(int offset, int devSelect);

  // FRAME LEVEL FUNCTIONS
  void grabFrame(int chan, int src);
  int retrieveFrame(int chan, uchar* data);
  void readFrame(int offset, uchar* data, int step) const; 
  void writeFrame(int offset, uchar* data, int step);
  void clearFrame(int offset, const uchar val=0);

  // UTILITY FUNCTIONS
  /// Return offset of a named CSR, or negative if unrecognized
  int str2csr(const char* csrName);
  /// Return string holding the name of the CSR corresponding to the offset given
  const char* csr2str(int csr);

 private:
  int devfile;
  int* csrBase;
  int* memBase;
  bool noDMA;
  bool noINT;
  bool isOpen;
  int chanSrc[4];

};
#endif

