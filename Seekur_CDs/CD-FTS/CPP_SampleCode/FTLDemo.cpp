/*********************************************************************
 * Software License Agreement (BSD License)
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

/*
 * FTLDemo
 * Version 1.2	June 12th, 2013    
 *
 * Test program to show how to interface with the force torque sensor Schunk FTL. 
 * The program has to be started in a terminal window.
 * Written and tested on Ubuntu 11.10, gcc
 * Libraries: boost
 *
 * Functionalities: 
 * 	- establish a connection to the sensor via a virtual com port of the USB2CAN driver. (to be set in RS232CAN.cpp, preset is /dev/ttyUSB0)
 * 	- establish a communication loop and request force measurements
 * 	- show the measurements on the terminal
 *
 * Usage:
 * 1. Start with ./FTLDemo
 *
 */



#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"
#include <boost/lexical_cast.hpp>
#include <iostream>

#include "RS232CAN.h"


using namespace std;





class FTLDemo
{
public:
  	FTLDemo();
	void DoComm(void);

private:
  	
	void ApplyCalibMatrix();
	void Wait(int ms);

	RS232CAN itf;			// provides the hardware interface to the sensors CAN bus via a virtual com port

	int sensorID;			// CAN ID of the sensor
	int cnt;

	double xyz[6];
	double raw[6];
	double rawBias[6];
	int startUpCnt;
	double temp;  
	double calibMat[36];

};

//********************************************
// Initialization
FTLDemo::FTLDemo()
{
	sensorID = 80;			// 80 (0x50) is standard CAN ID
	cnt = 0;

	for(int i=0; i<6; i++){
		xyz[i] = 0.0;
		raw[i] = 0.0;
		rawBias[i] = 0.0;
	}
	temp = 0.0;

	// the calibration matrix has to be adapted to the specific sensor!
	// It can be found on the CD root directory, on the quality assurance sheet, and in the file configuration.xml of SensorViz
	calibMat[0] = 0.00074; calibMat[1] = 0.15425; calibMat[2] = -0.01197; calibMat[3] = 0.26927; calibMat[4] = -0.00961; calibMat[5] = 0.16185;
	calibMat[6] = -0.00297; calibMat[7] = 0.18613; calibMat[8] = -0.00195; calibMat[9] = -0.19274; calibMat[10] = 0.01098; calibMat[11] = -0.39257;
	calibMat[12] = -0.07788; calibMat[13] = -0.07254; calibMat[14] = 0.08492; calibMat[15] = -0.02224; calibMat[16] = -0.07898; calibMat[17] = 0.04599;
	calibMat[18] = -0.05767; calibMat[19] = -0.17203; calibMat[20] = 1.16446; calibMat[21] = -0.14635; calibMat[22] = 1.29975; calibMat[23] = -0.04851;
	calibMat[24] = 1.76953; calibMat[25] = 1.36467; calibMat[26] = 0.15277; calibMat[27] = 0.39187; calibMat[28] = -0.37144; calibMat[29] = -0.89138;
	calibMat[30] = -0.04327; calibMat[31] = -1.33824; calibMat[32] = -0.10269; calibMat[33] = 2.66601; calibMat[34] = -0.26077; calibMat[35] = -0.65777;
	
	startUpCnt = 15;	// 15 values will be used to gather the bias
	itf.Connect();		// connect with the USB2CAN interface
}



//********************************************
// Main communication loop. 
// Reads the sensor values and shows them on the terminal
void FTLDemo::DoComm(){

	// storage necessary for the CAN communication
	int l = 2;				// length for request: 2 byte. When the length is not correct, the sensor will not answer!
	unsigned char data[8] = {7, 1, 0, 0, 0, 0, 0, 0};	
	int tmpTC = 0;

	// Request measurements with an incrementing message identifier
	data[0] = 7;						// Request measurements
	data[1] = cnt;
	itf.WriteMsg(sensorID, l, data);			// write the message to the joint controller
	Wait(10);						// wait a short time to avoid a crowded bus

	// read the answers from the sensor, two messages
	itf.GetMsg(sensorID+1, &l, data);			// get the last message that has been received
	tmpTC = data[0];					// byte 0 contains the timecode from above and error bits	
	raw[0] = 256 * data[1] + data[2];			// three raw measureemnts in this messages, three in the next
	raw[1] = 256.0 * data[3] + data[4];
	raw[2] = 256.0 * data[5] + data[6];
	temp = 256.0 * data[7];					// and the temp-highbyte

	itf.GetMsg(sensorID+2, &l, data);	
	raw[3] = 256.0 * data[1] + data[2];
	raw[4] = 256.0 * data[3] + data[4];
	raw[5] = 256.0 * data[5] + data[6];
	temp += data[7];					// temp low-byte
	temp = 0.12 * temp - 240.0;				

	// Error Check:
	// tmpTC is the first byte of the answers
	// it contains the time code (second byte sent to the sensor) and
	// error messages: bit 6 for Overload, bit 7 for general sensor error
        if (tmpTC > 127)
        {
		std::cout << "!!! Sensor Error !!!"<< std::endl;               
		tmpTC -= 127;
        }
    	if (tmpTC > 63)
    	{
		std::cout << "!!! Overload !!!"<< std::endl;               
        	tmpTC -= 63;
    	}


	
	// Use the first 15 measurements to compute the bias value
	if(startUpCnt > 0){
		std::cout << "Setting bias..." << std::endl;
		if(startUpCnt <= 10){		// the first 5 measurements are not used

			for(int i=0; i<6; i++)		
				rawBias[i] += raw[i];
		
			if(startUpCnt == 1){
				for(int i=0; i<6; i++)
					rawBias[i] /= 10;
				std::cout<<"Bias values: "<<rawBias[0]<<" - "<<rawBias[1]<<" - "<<rawBias[2]<<" - "<<rawBias[3]<<" - "<<rawBias[4]<<" - "<<rawBias[5]<<std::endl;
			}		
		}
		startUpCnt--;

	// but then generate the XYZ values
	// first subtract the bias from the current values,
	// then multiply with the calibration matrix
	}else{
		for(int i=0; i<6; i++)
			raw[i] -= rawBias[i];
		ApplyCalibMatrix();
		//std::cout << raw[0] << " - " << raw[1] << " - "  << raw[2] << " - "  << raw[3] << " - "  << raw[4] << " - "  << raw[5] << std::endl;
		std::cout << "   x: " << xyz[0] << "   y: " << xyz[1] << "   z: " << xyz[2];
		std::cout << "  rx: " << xyz[3] << "  ry: " << xyz[4] << "  rz: " << xyz[5] << std::endl;
	}

	cnt++;							//increment the message identifier 0-127
	if(cnt >= 64)
		cnt = 0;


	return;
}



//*************************************************+
// Matrix multiplication: xyz = caliMat * raw
void FTLDemo::ApplyCalibMatrix(){

	for(int i=0; i<6; i++){
		xyz[i] = 0.0;
		for(int j=0; j<6; j++){
			xyz[i] += calibMat[6*i+j] * raw[j]; 
		}
	}
}




//********************************************
void FTLDemo::Wait(int ms){
	using namespace boost::posix_time;
	ptime start, now;
	time_duration passed;

	start = microsec_clock::universal_time();
	now = microsec_clock::universal_time();
	passed = now - start;
	while( passed.total_milliseconds() < ms){
		now = microsec_clock::universal_time();
		passed = now - start;
	}

}




//********************************************
int main( int argc, char** argv )
{
		
	using namespace boost::posix_time;
	ptime start, now;
	time_duration passed;
	
	FTLDemo mySensor;	
  	
	
	while (true)			// this is the main loop
	{				// but all interesting things are done in DoComm()
		start = microsec_clock::universal_time();
		mySensor.DoComm();
		now = microsec_clock::universal_time();
		passed = now - start;

		while( passed.total_milliseconds() < 100){	// set the cycle time here
			now = microsec_clock::universal_time();
			passed = now - start;

		}

	}



}



