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
include ../etc/globals.mk

# Create a list of all subdirectories with Makefiles in them
TARGETS := $(shell for file in `ls`; do if [ -d "$$file" -a -f "$$file/Makefile" ]; then echo $$file; fi; done)
# Old way of listing targets manually left for now
SRC_TARGETS ?= libfrdev libfrold libfrcam libfrsim util calib model apps demo_lin drvr_v4l drvr_egl jtag refdes

.PHONY: all test install uninstall clean distclean

all:
	for dir in $(TARGETS); do $(MAKE) -C $$dir ; done

test:
	for dir in $(TARGETS); do $(MAKE) -C $$dir test; done

install:
	for dir in $(TARGETS); do $(MAKE) -C $$dir install; done

uninstall:
	for dir in $(TARGETS); do $(MAKE) -C $$dir uninstall; done

clean:
	for dir in $(TARGETS); do $(MAKE) -C $$dir clean; done

distclean:
	for dir in $(TARGETS); do $(MAKE) -C $$dir distclean; done


