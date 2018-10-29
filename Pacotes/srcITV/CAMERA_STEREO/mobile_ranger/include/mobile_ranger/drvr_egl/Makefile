#****************************************************************************
# Copyright (c) 2006-2009 by Focus Robotics Inc.
#
# All rights reserved. No part of this design may be reproduced stored
# in a retrieval system, or transmitted, in any form or by any means,
# electronic, mechanical, photocopying, recording, or otherwise, without
# prior written permission of Focus Robotics, Inc.
#
# Proprietary and Confidential
#
# Creation_Date :  Sat Jun 27 2006
# Created by    :  Andrew Worcester
#
#*****************************************************************************

obj-m	:= eagle.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

.PHONY: all test install uninstall clean distclean

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

test:
	echo "No test target"

install:
	echo "No install target"

uninstall:
	echo "No uninstall target"

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions
	rm -f Module.symvers modules.order

distclean: clean ;

