###############################################################################
## Copyright (c) 2005-2009 Focus Robotics. All rights reserved. 
##
## This program is free software; you can redistribute it and or modify it
## under the terms of the GNU General Public License as published by the 
## Free Software Foundation; either version 2 of the License, or (at your 
## option) any later version.
##
## This program is distributed in the hope that it will be useful, but 
## WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
## General Public License for more details.
##
###############################################################################
LIBFRCAM = true

include ../../etc/globals.mk
LIBFRCAM_TYPE ?= esc
LIBFRCAM_BUILD ?= fr3 esc

# Configuration section -------------------------------------------------------
ifdef DEVSYS_MODE
CXXFLAGS += -DDEVSYS_MODE
endif

SOURCE_DEPS = $(patsubst %.cpp, objs/%.d, $(notdir $(LIBFRCAM_SRCS)))
INC = $(OPENCV_INCLUDES) $(LIBFRDEV_INCLUDES)

# List library files here (we can split out into multiple library files later)
LIBFILES_FR3 = frchan_fr3.cpp frvcam_fr3.cpp frmap.cpp frstat.cpp fravi.cpp frvis.cpp
LIBFILES_ESC = frcam_egl.cpp frmap.cpp frstat.cpp fravi.cpp frvis.cpp
LIBFILES_WDM = 

LIBOFILES_FR3 = $(LIBFILES_FR3:%.cpp=objs/%.o) 
LIBOFILES_ESC = $(LIBFILES_ESC:%.cpp=objs/%.o) 
LIBS = $(notdir $(LIBFRCAM_LIBS))

all: $(LIBS) ;

mdebug: $(LIBS)
	echo $(LIBFRCAM_LIBS)

.PHONY: all test doc install uninstall clean distclean

# Generate automatic header dependencies ---------------------------------------
-include $(SOURCE_DEPS)


objs/%.d: %.c
	@set -e; rm -f $@; \
         $(CC) -M $(CXXFLAGS) $(INC) $< > $@.$$$$; \
         sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
         rm -f $@.$$$$

objs/%.d: %.cpp
	@set -e; rm -f $@; \
         $(CXX) -M $(CXXFLAGS) $(INC) $< > $@.$$$$; \
         sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
         rm -f $@.$$$$


# Rules -----------------------------------------------------------------------
objs/%.o : %.c
	$(CC) $(CXXFLAGS) $(PROFILE) $(INC) -c $< -o $@

objs/%.o : %.cpp
	$(CXX) $(CXXFLAGS) $(PROFILE) $(INC) -c $< -o $@

libfrcam_fr3.a: $(LIBOFILES_FR3)
	rm -f $@
	ar rvs $@ $(LIBOFILES_FR3)

libfrcam_esc.a: $(LIBOFILES_ESC)
	rm -f $@
	ar rvs $@ $(LIBOFILES_ESC)

libfrcam.a:
	rm -f libfrcam.a
	ln -sf libfrcam_$(LIBFRCAM_TYPE).a libfrcam.a

# Final targets ---------------------------------------------------------------

test:
	echo "No test target"

install: $(LIBS) 
	cp -a $(LIBFRCAM_LIBS) $(LIBFRCAM_INST)/lib
	cp -a $(LIBFRCAM_HDRS) $(LIBFRCAM_INST)/include/

uninstall:
	rm -f $(LIBFRCAM_LIBS:%.a=$(LIBFRCAM_INST)/lib/%.a)
	rm -f $(LIBFRCAM_HDRS:%.h=$(LIBFRCAM_INST)/include/%.h)

# A rule that generates documentation
doc: Doxyfile $(LIBFRCAM_SRCS) $(SOURCE_DEPS)
	doxygen

clean:
	rm -rf objs/* *.d.* doc $(LIBFRCAM_LIBS)

distclean: uninstall clean ;



