/**
 * @file /include/rosa_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rosa_gui_QNODE_HPP_
#define rosa_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include "ros/ros.h"
#include <string>
#include <QThread>
#include <QStringListModel>
#include "romansa/rosa_dynamixel_msg.h"
#include "../include/rosa_msg/readdata.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rosa_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	//bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	int Joint_Set[11];
	int Off_Set[11];
        int Velocity[11];
	int Position[11];
	int Current[7];
        int Temperature[4];
	int Present_XYZ[2];
	int Goal_XYZ[2];
	int Error_XYZ[2];
	int Present_Theta[2];
        char Status_Window[9];
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
        void rosShutdown();
	void updateStatus();

private:
	void ReadData(const rosa_msg::readdata::ConstPtr& msg);
	int init_argc;
	char** init_argv;
	//ros::Publisher chatter_publisher;
        QStringListModel logging_model;
};

}  // namespace rosa_gui

#endif /* rosa_gui_QNODE_HPP_ */
