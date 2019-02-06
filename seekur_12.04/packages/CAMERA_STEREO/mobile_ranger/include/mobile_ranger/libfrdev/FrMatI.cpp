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
 * Creation_Date:  Thu Mar 22 2007
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

#include "FrMatI.h"

FrMatI::FrMatI(int w, int h, int c) : width(w), height(h), nchans(c) {
  dptr = new int[height * width * nchans];
  stride = width * nchans; // stride is given in elements, at least for now
  dptrIsRef = false;
}

void FrMatI::setElem(int dat, int x, int y, int chan) {
  *(dptr + (y*stride) + (x*nchans) + chan) = dat;
}

int FrMatI::getElem(int x, int y, int chan) const {
  return *(dptr + (y*stride) + (x*nchans) + chan);
}

int FrMatI::getHeight() const { return height; }
int FrMatI::getWidth() const { return width; }
int FrMatI::getChans() const { return nchans; }
int FrMatI::getStride() const { return stride; }
int* FrMatI::getDptr(int row) const { 
  if(row) return dptr + stride*row;
  else return dptr; 
}

