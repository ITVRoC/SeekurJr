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
 * Creation_Date:  Wed Jan  3 2007
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
#include <stdio.h>  // for printf
#include <stdlib.h> // for exit, atoi

#include "FrOptArgs.h"

FrOptArgs::FrOptArgs(int argc, char** argv) {
  // Populate the list of standard args and setup defaults
  initArgs();
  // Load cmd line options
  parseCmdLine(argc, argv);
  // Load env options, only overwriting if not from cmd line
  // Load options file, only overwriting if not from cmd line or env
}

FrArgDescr* FrOptArgs::getArg(const char* argName) {
  // Search through arg list and return struct with matching name, or null if no
  // matches. The leading "--" may or may not be present in argName.
  for(unsigned int i=0; i<stdArgs.size(); i++) {
    if(stdArgs[i].name==argName) {
      return &stdArgs[i];
    }
  }
  return 0;
}

void FrOptArgs::pushArg(const char* name, int type, int origin, const char* val) {
  // alloc memory for new name and data
  // alloc memory for new struct and populate
  FrArgDescr a;
  a.name = name;
  a.type = type;
  a.origin = origin;
  a.val = val;
  // push onto list of standard args--will make a copy
  stdArgs.push_back(a);

  // FIXME: test to see if name already exists in the std arg list and just 
  // update vals if so, possibly issuing warning
  // This should only really be called by initArgs and there should never
  // be conflicts with existing args

}

// Sets up all standard args and their defaults (if any)
// This function basically pushes the whole list of standard args known to libfrdev
// type: 1=char*, 2=int, 3=float, or 4=bool (no arg for bool, just present or not, could support +/- notation also)
// origin: 1=not set, 2=cmd line, 3=env, 4=opt file, 5=default, 6=override; should it be an enum?
void FrOptArgs::initArgs() {
  pushArg("pci-device", 1, 1);
  pushArg("camera-number", 2, 5, "0");
  pushArg("calib-file", 1, 5, "/etc/fr/default.calib");
  pushArg("init-camera", 2, 5, "1");
}

void FrOptArgs::parseCmdLine(int argc, char** argv) {
  // loop through all cmd line args, checking for std args
  // To find standard args: they must start with --
  // the rest of the string must exactly match a name in the stdArgs list
  // the type field of the arg determines whether the next arg is the value
  // if the type is 4 (bool) then there is no value, just the presence of the arg
  // If it doesn't pass all of these criteria, then push it onto extraArgs
  for(int i=1; i<argc; i++) {
    printf("Cmd Line Arg: %s ", argv[i]);
    std::string s = argv[i];
    std::string s1;
    if(s.size()>2) s1= s.substr(2);
    FrArgDescr* a;
    if(s.substr(0,2)=="--" && (a=getArg(s1.c_str()))) {
      a->origin = 2; // cmd line origin
      printf("added to stdArgs ");
      if(a->type != 4) {
	i++;
	a->val = argv[i];
	printf("with value %s", argv[i]);
      }
    } else {
      extraArgs.push_back(s);
      printf("added to extraArgs");
    }
    printf("\n");
  }
}

int FrOptArgs::checkArg(const char* argName) {
  // getArg() on argName and return origin value
  FrArgDescr* a;
  if((a=getArg(argName))) {
    return a->origin;
  }
  // return 0 if getArg() doesn't match anything
  return 0;
}

void FrOptArgs::setArg(const char* argName, const char* val) {
  FrArgDescr* a;
  if((a=getArg(argName))) {
    a->origin = 6;
    a->val = val;
  } else {
    printf("WARNING: FrOptArgs: Trying to set a non-existant arg");
  }
}

int FrOptArgs::getIntArg(const char* argName) {
  // getArg() on argName and return value if type is int
  FrArgDescr* a;
  if((a=getArg(argName))) {
    if(a->type==2) {
      return atoi(a->val.c_str());
    } else {
      printf("WARNING: trying to convert a non-int arg to an int\n");
    }
  }
  return 0;
}

const char* FrOptArgs::getCharArg(const char* argName) {
  FrArgDescr* a;
  if((a=getArg(argName))) {
    return a->val.c_str();
  }
  // return null string if getArg() doesn't match anything
  return "";
}

float FrOptArgs::getFloatArg(const char* argName) {
  printf("WARNING: FrOptArgs not fully implemented\n");
  return 0;
}

bool FrOptArgs::getFlagArg(const char* argName) {
  printf("WARNING: FrOptArgs not fully implemented\n");
  return 0;
}

int FrOptArgs::getPosIntArg(unsigned int argnum) {
  if(extraArgs.size()>argnum)
    return atoi(extraArgs[argnum].c_str());
  else
    return 0;
}

const char* FrOptArgs::getPosCharArg(unsigned int argnum) {
  if(extraArgs.size()>argnum)
    return extraArgs[argnum].c_str();
  else
    return "";
}

float FrOptArgs::getPosFloatArg(unsigned int argnum) {
  printf("WARNING: FrOptArgs not fully implemented\n");
  return 0;
}


