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

#include "FrMatF.h"

FrMatF::FrMatF(int w, int h, int c) : width(w), height(h), nchans(c) {
  dptr = new float[height * width * nchans];
  stride = width * nchans; // stride is given in elements, at least for now
  dptrIsRef = false;
}

FrMatF::~FrMatF() {
  if(!dptrIsRef)
    delete[] dptr;
}

void FrMatF::setElem(float dat, int x, int y, int chan) {
  *(dptr + (y*stride) + (x*nchans) + chan) = dat;
}

float FrMatF::getElem(int x, int y, int chan) const {
  return *(dptr + (y*stride) + (x*nchans) + chan);
}

int FrMatF::getHeight() const { return height; }
int FrMatF::getWidth() const { return width; }
int FrMatF::getChans() const { return nchans; }
int FrMatF::getStride() const { return stride; }

float* FrMatF::getDptr(int row) const { 
  if(row) return dptr + stride*row;
  else return dptr; 
}

