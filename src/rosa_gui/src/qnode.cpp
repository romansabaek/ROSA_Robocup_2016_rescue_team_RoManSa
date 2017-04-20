/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <QtGui>
#include <QMessageBox>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <getopt.h>
#include <fcntl.h>

#include "../include/rosa_gui/qnode.hpp"
#include "../include/romansa/rosa_dynamixel_msg.h"
#include "../include/rosa_msg/readdata.h"
#include "../include/rosa_gui/main_window.hpp"

#define PKT_RTN_DELAY_US	5000
#define CONTROL_PERIOD		10000 // usec (Large value is more slow)
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rosa_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/
ros::Publisher gui_pub;
ros::Subscriber gui_sub;

void QNode::ReadData(const rosa_msg::readdata::ConstPtr& msg)
{
  
		
		for(int i=0;i< msg->DX_JOINT.size();i++)
		{
		  Joint_Set[i] = msg->DX_JOINT[i];
	          usleep(PKT_RTN_DELAY_US);		
                }
		for(int i=0;i< msg->DX_OFFSET.size();i++)
		{
		  Off_Set[i] = msg->DX_OFFSET[i];
		  usleep(PKT_RTN_DELAY_US);

                }
		for(int i=0;i< msg->DX_SPEED.size();i++)
		{
		  Velocity[i] = msg->DX_SPEED[i];
		  usleep(PKT_RTN_DELAY_US);

                }
		for(int i=0;i< msg->DX_POSI.size();i++)
		{
		  Position[i] = msg->DX_POSI[i];
		  usleep(PKT_RTN_DELAY_US);

                }
		for(int i=0;i< msg->DX_CURRENT.size();i++)
		{
		  Current[i] = msg->DX_CURRENT[i];
		  usleep(PKT_RTN_DELAY_US);

                }
		for(int i=0;i< msg->DX_TEMPERATURE.size();i++)
		{
		  Temperature[i] = msg->DX_TEMPERATURE[i];
		  usleep(PKT_RTN_DELAY_US);

                }
		
		
           
}




QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"rosa_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle nh;
	// Add your ros communications here.
        gui_pub = nh.advertise<romansa::rosa_dynamixel_msg>("rosa_dynamixel", 100);
	gui_sub = nh.subscribe<rosa_msg::readdata>("read_data", 100, &QNode::ReadData,this);
	start();
	log(Info,"Node Connected to ROSCORE");
	return true;
}


void QNode::run() {
    while ( ros::ok() ) {

        ros::spinOnce();
    }
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace rosa_gui
