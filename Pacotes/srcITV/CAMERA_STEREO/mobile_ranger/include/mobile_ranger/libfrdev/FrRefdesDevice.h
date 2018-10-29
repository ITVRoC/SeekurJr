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
#ifndef FRREFDESDEVICE_H
#define FRREFDESDEVICE_H

#include "FrRefdesDeviceRegs.h"

class FrRefdesDevice {
 public:
  // CONSTRUCTOR/DESTRUCTOR
  FrRefdesDevice(int id=0, int doInit=1);
  ~FrRefdesDevice();

  // INIT FUNCTIONS
  int findPCIDev(int vendor_id, int device_id, int *domain, int *bus, int *slot);
  int openSysfs(int domain, int bus, int slot);

  // LOW LEVEL FUNCTIONS
  void setCSR(int offset, int data);
  int getCSR(int offset) const;
  int waitCSR(int offset, int data, int mask=-1, int timeout=50) const;
  void writeMEM(int offset, int length, const int* data);
  void readMEM(int offset, int length, int* data) const;  

  // HIGHER LEVEL FUNCTIONS
  void readFrame(int offset, char* data, int step) const; 
  void writeFrame(int offset, const char* data, int step);
  void clearFrame(int offset, const char data=0);
  int writeWordI2C(int offset, int data, int dev);
  int readWordI2C(int offset, int *data, int dev);

 private:
  int devfile;
  int* csrBase;
  int* memBase;
};
#endif

