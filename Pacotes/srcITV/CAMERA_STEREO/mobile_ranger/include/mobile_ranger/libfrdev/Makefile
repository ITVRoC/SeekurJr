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
# Creation_Date :  Mon Jan  1 2007
# Created by    :  Andrew Worcester
#
#*****************************************************************************
LIBFRDEV = true
include ../../etc/globals.mk

# This library depends on the following external libraries
# std/c library -- always in link/include path
# opencv -- get link/include details from globals.mk
# netpbm -- assume include path, should get it from globals.mk  FIXME
# It creates one static library--should I consider making a .so???

LIBFRDEV_SRCS = ${wildcard *.cpp} 
LIBFRDEV_HDRS = $(wildcard *.h)
LIBFRDEV_OBJS = $(LIBFRDEV_SRCS:%.cpp=objs/%.o)
LIBFRDEV_LIBFILES = $(notdir $(LIBFRDEV_LIBS))
DEPFILES = $(LIBFRDEV_SRCS:%.cpp=objs/%.d)
INCLUDES = $(OPENCV_INCLUDES) -I/usr/include/netpbm

all: $(LIBFRDEV_LIBS) ;

.PHONY : all test doc install uninstall clean distclean

-include $(DEPFILES)

objs/%.d: %.cpp
	@rm -f $@; \
	$(CXX) -MM $(CXXFLAGS) $(INCLUDES) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

objs/%.o : %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Generate the library
$(LIBFRDEV_LIBS): $(LIBFRDEV_OBJS)
	rm -f $@
	ar cq $@ $(LIBFRDEV_OBJS)

# Run unit tests
test:
	echo "No test target"

# Generate documentation
doc: Doxyfile ${LIBFRDEV_SRCS}
	doxygen

# Install lib and header files
install: $(LIBFRDEV_LIBS)
	cp $(LIBFRDEV_LIBS) $(LIBFRDEV_INST)/lib
	cp $(LIBFRDEV_HDRS) $(LIBFRDEV_INST)/include

# Uninstall lib and header files
uninstall:
	rm -f $(LIBFRDEV_LIBFILES:%=$(LIBFRDEV_INST)/lib/%)
	rm -f $(LIBFRDEV_HDRS:%=$(LIBFRDEV_INST)/include/%)

clean:
	rm -rf objs/* doc $(LIBFRDEV_LIBS)

distclean: uninstall clean ;



