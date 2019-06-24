#include "ros/ros.h"
#include <geometry_msgs/WrenchStamped.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>


#include "FTSLWA.h"


int main(int argc, char **argv) {
    using namespace boost::posix_time;

    ros::init(argc, argv, "force_sensor");

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    std::string fpath;
    nh.getParam("calibration_matrix", fpath);
    FTSLWA mySensor;
    mySensor.apply_bias = true;

    if (!mySensor.loadCalibration("/home/andre/catkin_ws/src/fts_lwa/src/force_sensor_calib.dat")) {
        puts("error loading calibration data");
        return -2;
    }
    // TO DO: Change to can1 before test on SeekurJr and change to accept can0 as a parameter
    const char *canif = "can0";
    if (argc > 1) {
        if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
            puts("Usage: force_sensor [can_interface]");
            return 0;
        }
        canif = argv[1];
    }

    if (mySensor.Connect(canif)) {
        puts("connected");
    } else {
        puts("error opening connection to sensor or CANBUS");
        return -1;
    }


    ptime last = microsec_clock::universal_time();
    ptime now;
    time_duration passed;


    std::string frame_id = "force_torque_sensor";
    nh.getParam("frame_id", frame_id);

    ros::Publisher force_sensor_pub = n.advertise<geometry_msgs::WrenchStamped>("force_torque", 1);


    while (ros::ok()) {
        mySensor.DoComm(); // checks for new messages and stores them, periodically sends request for new messages
        now = microsec_clock::universal_time();
        passed = now - last;
        if (passed.total_milliseconds() >= 250) {
            last = microsec_clock::universal_time();
            geometry_msgs::WrenchStampedPtr msg(new geometry_msgs::WrenchStamped);


            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = frame_id;
            msg->wrench.force.x = mySensor.NewXyz[0];
            msg->wrench.force.y = mySensor.NewXyz[1];
            msg->wrench.force.z = mySensor.NewXyz[2];
            msg->wrench.torque.x = mySensor.NewXyz[3];
            msg->wrench.torque.y = mySensor.NewXyz[4];
            msg->wrench.torque.z = mySensor.NewXyz[5];
            force_sensor_pub.publish(msg);

            fflush(stdout);
            ros::spinOnce();
        }
    }

    ros::shutdown();


}



