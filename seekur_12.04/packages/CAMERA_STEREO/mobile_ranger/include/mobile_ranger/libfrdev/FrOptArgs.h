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
 * Creation_Date:  Fri Dec 29 2006
 * 
 * Brief Description:
 * Common command line argument parsing for libfrdev
 * 
 * Functionality:
 * Creates a full list of control arguments to the program by parsing the 
 * command line and combining that with options that may be set in the 
 * environment or in an options file or as a default value.
 * 
 * Issues:
 * FIXME: maybe getCharArg should just be getArg since no conversion is done?
 * 
 * Limitations:
 * 
 * Testing:
 *
 * Standard Options:
 *  Device Selection Options:
 *   --pci-dev %s (%x:%x:%x)
 *   --mem-off %s (%x:%x)
 *   --cam-num %d (0-7, default 0)
 *
 *  Camera Control Options:
 *
 *  Processor Control Options:
 *
 *  Output Selection Options:
 * 
 ******************************************************************************/

#ifndef FROPTARGS_H
#define FROPTARGS_H

#include <vector>
#include <string>

struct FrArgDescr {
  std::string name;
  int type; // 1=char*, 2=int, 3=float, or 4=bool (no arg for bool, just present or not, could support +/- notation also)
  int origin; // 1=not set, 2=cmd line, 3=env, 4=opt file, 5=default; should it be an enum?
  std::string val;
};

class FrOptArgs {
 public:
  // Constructor
  // FIXME: would need to pass a pointer to argc if I wanted to modify args list--would that be useful?
  FrOptArgs(int argc, char** argv);
  // Function to check if we have a certain standard arg
  // returns 0 if not set, 1, 2, 3 or 4 depending on where it was set
  // 1 for cmd line, 2 for environment, 3 for option file, 4 for default val
  int checkArg(const char* argName);
  
  // Function to check the value associated with an arg (int, string, float, etc)
  // Throws an exception of the arg doesn't exist or can't be converted to the given format
  // Is there a better way of selecting standard args then by a string name?
  int getIntArg(const char* argName);
  const char* getCharArg(const char* argName);
  float getFloatArg(const char* argName);
  bool getFlagArg(const char* argName);
  void setArg(const char* argName, const char* val); // overload for all types, return error if argname not matched or wrong type
  // Probably also allow setting extra args, just for consistancy
  void setArg(unsigned int argnum, const char* val); // error to try to set a non-existant extra arg

  // Function to get 1st, 2nd, etc non-standard command line arg; called positional args
  // Throws an exception of the arg doesn't exist or can't be converted to the given format
  int checkPosArg(); // Returns the number of extra args available; extra args are all from the cmd line
  int getPosIntArg(unsigned int argnum);
  const char* getPosCharArg(unsigned int argnum);
  float getPosFloatArg(unsigned int argnum);

  // Enable/disable checking of unused args
  // some sort of call that tells how many standard args and how many extra args
  // were never used via a get call to warn the user that an option they set is wrong

  // Maybe add some control to how args are taken from option files or from the environment
  // Option files and env vars aren't read at all for now.

 protected:
  // Function that iterates the list of std args to find the match, used by all
  // getArg and checkArg functions. Return null if not matched.
  FrArgDescr* getArg(const char* argName);

  // Initially, all known args are pushed either with default values or as not set
  // This initial list can be updated from the cmd line or files or whatever
  void pushArg(const char* name, int type, int origin, const char* val="");

  // Sets up all standard args and their defaults (if any)
  void initArgs();

  // Function to parse all command line args and add standard options to stdArgs and 
  // unknown options to extraArgs.
  // What about support for stripping off standard options and leaving extra args
  // in the argc/argv list??
  void parseCmdLine(int argc, char** argv);

  // Add functions to add args from an options file or the environment

  // our local copy of all args--is it needed?

  // Hash table of standard args indexed by name and their default values (if any)
  std::vector<FrArgDescr> stdArgs;

  // array of extra args indexed by number
  //char* extraArgs[32];
  std::vector<std::string> extraArgs;

};
#endif
