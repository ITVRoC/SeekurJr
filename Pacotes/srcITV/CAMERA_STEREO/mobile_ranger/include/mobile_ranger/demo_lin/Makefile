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

.PHONY: all test install uninstall clean distclean

INCLUDES        += $(LIBFRCAM_INCLUDES) $(LIBFRDEV_INCLUDES) $(OPENCV_INCLUDES) -I/usr/include/netpbm
LDFLAGS		+= $(OPENCV_LDFLAGS)
LDLIBS		+= $(OPENCV_LDLIBS) -lnetpbm
CXXFLAGS	+= -g
CFLAGS		= -g

#SRCS		=  ${wildcard *.c} 
# Don't compile v4l2_simple.c for now since I don't have v4l2 support in my
# kernel:
SRCS		=  opencv_simple.c opencv_simple_color.c opencv_simple_color_mouse.c opencv_simple_mouse.c vcam_corr_img.c
DEPS		= $(SRCS:%.c=%.d)
OBJS 		= $(SRCS:%.c=%.o)
BINS 		= $(SRCS:%.c=%)

CXX_SRCS	= ${wildcard *.cpp}
CXX_DEPS	= $(CXX_SRCS:%.cpp=%.d)
CXX_OBJS 	= $(CXX_SRCS:%.cpp=%.o)
CXX_BINS 	= $(CXX_SRCS:%.cpp=%)

ALL_BINS	= $(CXX_BINS) $(BINS)

all: $(ALL_BINS)

-include $(CXX_DEPS) $(DEPS)

%.d: %.cpp
	@rm -f $@; \
	$(CXX) -MM $(CXXFLAGS) $(INCLUDES) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

%.d: %.c
	@rm -f $@; \
	$(CC) -MM $(CFLAGS) $(INCLUDES) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

%.o : %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $<

%.o : %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $<

.PHONY : clean all install

$(CXX_BINS) : % : %.o $(LIBFRCAM_LIBS) $(LIBFRCAM_DEPS)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(BINS) : % : %.o $(LIBFRCAM_LIBS) $(LIBFRCAM_DEPS)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

test:
	echo "No test target"

install: $(ALL_BINS)
	cp -a $(ALL_BINS) $(SANDBOX_ROOT)/bin

uninstall:
	rm -f $(ALL_BINS:%=$(SANDBOX_ROOT)/bin/%)

clean:
	rm -rf *.o *.d $(BINS) $(CXX_BINS)

distclean: uninstall clean ;


