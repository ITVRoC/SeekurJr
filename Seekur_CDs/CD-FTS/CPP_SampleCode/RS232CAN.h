/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

using namespace::boost::asio;



struct msg{
	int id;				// message id
	int length;			// length of data part
	unsigned char data[8];		// data
	long time;			// receive time
};


class RS232CAN
{
public:
  	RS232CAN();
	bool Connect(void);
	void Disconnect(void);
	void WriteMsg(int id, int length, unsigned  char* data);
	int EvaluateBuffer(unsigned char* buf);
	int GetMsg(int id, int *length, unsigned  char* data);

	msg msgBuffer[256];
	
	bool active; // remains true while this object is still operating
	boost::asio::serial_port *port; // the serial port this instance is connected to

  	
private:
  

};



