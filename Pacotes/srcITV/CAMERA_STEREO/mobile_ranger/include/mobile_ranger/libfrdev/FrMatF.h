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
 * Matrix structure with floating point elements
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
#ifndef FRMATF_H
#define FRMATF_H

class FrMatF {
 public:
  // Constructor & Destructor
  FrMatF(int w, int h, int c=1);
  ~FrMatF();
  // Note: may want to add an easy way to create from a cvmat structure when using opencv

  // Access functions should fail if requesting pixel outside the image
  // Since this isn't necessarily an image type, should it be setElem(), getElem()?
  void setElem(float dat, int x, int y, int chan=0);
  float getElem(int x, int y, int chan=0) const;

  // Functions to get image information. This data can't be set after the image 
  // is allocated so there are no corresponding set functions.
  // A data array can be reshaped in some cases, though, so the reshape or setROI
  // functions might change what is returned here.
  int getHeight() const;
  int getWidth() const;
  int getChans() const;
  int getStride() const; // normally not needed, but needed if you grab the Dptr for direct access
  float* getDptr(int row=0) const;

  // Operations: add, sub, mult, div, equality, svd, misc linear algebra, concatenate, etc.

 protected:

 private:
  int width;      // Width in pixels
  int height;     // Height in pixels
  int stride;     // Distance between consecutive rows in bytes
  int nchans;     // Number of channels in the image--essentially the size of the 3rd dimentions
                  // what about support for n dimensional arrays?
  float* dptr;    // Pointer to the actual matrix data
  bool dptrIsRef;// Set if we reference image data rather then alloc it in the constructor

};

#endif

