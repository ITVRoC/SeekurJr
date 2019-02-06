/****************************************************************************
 * Copyright (c) 2006 by Focus Robotics.
 *
 * All rights reserved. No part of this design may be reproduced stored
 * in a retrieval system, or transmitted, in any form or by any means,
 * electronic, mechanical, photocopying, recording, or otherwise, without
 * prior written permission of Focus Robotics, Inc.
 *
 * Proprietary and Confidential
 *
 * Created By   :  Andrew Worcester
 * Creation_Date:  Tue Dec 26 2006
 * 
 * Brief Description:
 * FrMsgLog is a singleton class which allows us to have very good control over
 * which messages get printed and where they get printed. 
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

#ifndef FRMSGLOG_H
#define FRMSGLOG_H

#include <string>
#include <vector>

class FrMsgLog {
 public:
  // The Instance() function enforces the singleton constraint
  static FrMsgLog* Instance();

  // The log() function actually creates log messages.
  // I may also want to set up operator << to support c++ style message IO
  void log(const char* loc, int level, const char* fmt, ...);

  // Several function are needed to control logging. We need to control where
  // the messages go: to the screen in stdout or stderr, or to a file, or 
  // where ever. We also need to control which messages actually get printed.
  // There is a level and messages can be printed based on level alone, or they
  // can be printed based on the location string alone, or they can be printed 
  // based on both where certain areas get printed at one level, and other areas
  // get printed at a different level. The last behavior may not be added for a
  // while, since it's not critical until the code gets quite large.
  // For now just print to the screen below a certain level.
  // 1=error/critical, 2=important/danger, 3=warning, 4=info/note, 5=debug
  // I can add additional levels higher, lower or in-between those any time.
  void setLogLevel(int level);
  void disableLogLoc(const char* loc);
  void enableLogLoc(const char* loc);
  void disableLogLocAll();
  void enableLogLocAll();
  void setLogFile(const char* filename);
  void setLogMsgOutput(); // how to set up? some msgs to stdout, some to stderr?
  // Can I log to both a file and to stdout? Maybe to a serial port?

 protected:
  FrMsgLog();
  FrMsgLog(const FrMsgLog&);
  FrMsgLog& operator=(const FrMsgLog&);

 private:
  static FrMsgLog* logp;
  int msgThreshold;
  std::vector<std::string> enLoc; // maybe this should be a map: list of loc vals to print
  // actually, we really want a threshold for each loc, so we need a different container anyway
  // any loc not in the list uses the global threshold, but any loc in the list uses the
  // threshold placed with it in the list.
};

#endif

