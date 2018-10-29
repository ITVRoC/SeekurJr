#****************************************************************************
# Copyright (c) 2007-2009 by Focus Robotics Inc.
#
# All rights reserved. No part of this design may be reproduced stored
# in a retrieval system, or transmitted, in any form or by any means,
# electronic, mechanical, photocopying, recording, or otherwise, without
# prior written permission of Focus Robotics, Inc.
#
# Proprietary and Confidential
#
# Creation_Date :  Tue Jan  2 2007
# Created by    :  Andrew Worcester
#
#*****************************************************************************
include ../../etc/globals.mk

INCLUDES        += $(LIBFRDEV_INCLUDES) $(OPENCV_INCLUDES)
LDFLAGS		+= $(OPENCV_LDFLAGS)
LDLIBS		+= $(OPENCV_LDLIBS) -lnetpbm
CXXFLAGS	+= -g

SRCS		= ${wildcard *.c} 
DEPS		= $(SRCS:%.c=objs/%.d)
OBJS 		= $(SRCS:%.c=objs/%.o)
BINS 		= $(SRCS:%.c=%)

CXX_SRCS	= ${wildcard *.cpp} 
CXX_DEPS	= $(CXX_SRCS:%.cpp=objs/%.d)
CXX_OBJS 	= $(CXX_SRCS:%.cpp=objs/%.o)
CXX_BINS 	= $(CXX_SRCS:%.cpp=%)

ALL_BINS	= $(CXX_BINS) $(BINS) demo_scam_opencv_devsys

.PHONY: all test install uninstall clean distclean

all: $(ALL_BINS)

-include $(CXX_DEPS) $(DEPS)

objs/%.d: %.cpp
	@rm -f $@; \
	$(CXX) -MM $(CXXFLAGS) $(INCLUDES) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

objs/%.d: %.c
	@set -e; rm -f $@; \
	$(CC) -MM $(CFLAGS) $(INCLUDES) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

objs/%.o : %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

objs/%.o : %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@


objs/demo_scam_opencv_devsys.o: demo_scam_opencv.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -DDEVSYS_MODE=1 -c $< -o $@

demo_scam_opencv_devsys: objs/demo_scam_opencv_devsys.o $(LIBFRDEV_LIBS)
	$(CXX) $(LDFLAGS) $(LDLIBS) $^ -o $@

$(CXX_BINS) : % : objs/%.o $(LIBFRDEV_LIBS)
	$(CXX) $(LDFLAGS) $(LDLIBS) $^ -o $@

$(BINS) : % : objs/%.o 
	$(CC) $(LDFLAGS) $(LDLIBS) $^ -o $@

test:
	echo "No test targets."

install: $(ALL_BINS)
	cp -a $(ALL_BINS) $(INSTALL_ROOT)/bin

uninstall:
	rm -f $(ALL_BINS:%=$(INSTALL_ROOT)/bin/%)

clean:
	rm -rf objs/* $(ALL_BINS)

distclean: uninstall clean ;




