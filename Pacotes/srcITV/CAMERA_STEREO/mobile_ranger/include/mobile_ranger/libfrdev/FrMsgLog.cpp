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
 * Creation_Date:  Fri Jan  5 2007
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
#include "FrMsgLog.h"
#include <stdio.h>
#include <stdarg.h>

FrMsgLog* FrMsgLog::logp = 0;

FrMsgLog* FrMsgLog::Instance() {
  if(logp==0) {
    // construct initial FrMsgLog
    logp = new FrMsgLog;
  }
  return logp;
}

void FrMsgLog::log(const char* loc, int level, const char* fmt, ...) {
  char msg[255];
  va_list ap;
  va_start(ap, fmt);
  vsprintf(msg, fmt, ap);
  va_end(ap);

  // For now, all I do is print the message if the log level is high enough
  if(msgThreshold>level) {
    printf("%s(%d): %s\n", loc, level, msg);
  } 
}

void FrMsgLog::setLogLevel(int level) {
  msgThreshold = level;
}

void FrMsgLog::disableLogLoc(const char* loc) {

}

void FrMsgLog::enableLogLoc(const char* loc) {

}

void FrMsgLog::disableLogLocAll() {

}

void FrMsgLog::enableLogLocAll() {

}

void FrMsgLog::setLogFile(const char* filename) {

}

void FrMsgLog::setLogMsgOutput() {

}

FrMsgLog::FrMsgLog() {
  msgThreshold = 999;
}


