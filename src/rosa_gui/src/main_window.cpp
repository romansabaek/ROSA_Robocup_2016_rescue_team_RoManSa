/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <vector>
#include "../include/rosa_gui/main_window.hpp"
#include "../include/rosa_gui/qnode.hpp"
//#include "romansa/rosa_dynamixel_msg.h"
#include "../include/romansa/rosa_dynamixel_msg.h"
#include "../include/rosa_msg/readdata.h"
#include "../include/rosa_gui/main_window.hpp"
#include "stdio.h"
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

#define PKT_RTN_DELAY_US	5000
#define CONTROL_PERIOD		10000 // usec (Large value is more slow)
/*****************************************************************************
** Namespaces
*****************************************************************************/

using namespace Qt;

namespace rosa_gui {



extern ros::Publisher gui_pub;
extern ros::Subscriber gui_sub;


int c[] = {0,};
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)

{
    ui.setupUi(this);
    //QObject::connect(&qnode, SIGNAL(updateStatus()), this, SLOT(:on_pushButton_clicked()));


//////////////////////////// dynamixel real_time status value ///////////////////////////////
    
    
    
    //ui.JointSet_1->setDigitCount(qnode.Joint_Set[0]);
    
//////////////////////////// dynamixel real_time status value ///////////////////////////////



    scene = new QGraphicsScene(this);

    pen_ora.setColor(QColor(255,94,0));
    pen_ora.setWidth(5);
    pen_yel.setColor(QColor(255,187,0));
    pen_yel.setWidth(5);
    pen_blk.setColor(Qt::black);
    pen_blk.setWidth(8);

    line[0] = new QGraphicsLineItem(-100,0,0,0);
    line[1] = new QGraphicsLineItem(-100,0,0,0);
    line[2] = new QGraphicsLineItem(80,0,180,0);
    line[3] = new QGraphicsLineItem(80,0,180,0);
    line[4] = new QGraphicsLineItem(0,0,80,0);

    for(int i=0; i<4;i++)
    {
        if(i%2 == 0)
           line[i]->setPen(pen_ora);
        else
           line[i]->setPen(pen_yel);
        scene->addItem(line[i]);
    }
    line[4]->setPen(pen_blk);
    scene->addItem(line[4]);

    ui.graphicsView->setScene(scene); //graphic view


    setWindowIcon(QIcon(":/images/icon.png"));


    qnode.init();
}

MainWindow::~MainWindow() {}




//void MainWindow::on_pushButton_clicked()
//{
//    printf("go Home \n");

//    romansa::rosa_dynamixel_msg test;
//    test.mode = romansa::rosa_dynamixel_msg::ROSA_DX_GOAL_POSITION;
//    gui_pub.publish(test);	
    //printf(" %d \n",c[0]);
//    ui.lcdNumber->display(qnode.z[0]);
    //ui.lcdNumber_2->display("1");
//}

//void MainWindow::on_pushButton_2_clicked()
//{
//    printf("go CGV \n");

//    romansa::rosa_dynamixel_msg test2;
//  test2.mode = romansa::rosa_dynamixel_msg::ROSA_DX_TORQUE_ON;
//    gui_pub.publish(test2);
    //ui.lcdNumber->display(qnode.x[0]);
    //ui.lcdNumber->display("2");
    //ui.lcdNumber_2->display("2");
//}

//void MainWindow::on_pushButton_3_clicked()
//{
//    printf("Read Data \n");

//    romansa::rosa_dynamixel_msg test3;
//    test3.mode = romansa::rosa_dynamixel_msg::ROSA_DX_READ;
//    gui_pub.publish(test3);
    //ui.lcdNumber->display(qnode.c[0]);
    //ui.lcdNumber->display("3");
    //ui.lcdNumber_2->display("3");
//}
//}


void MainWindow::on_InitializeButton_clicked()
{
    
}

void MainWindow::on_SafeEndButton_clicked()
{
    
}

void MainWindow::on_EmergencyButton_clicked()
{
    
}

void MainWindow::on_LegButton_1_clicked()
{
    
}

void MainWindow::on_LegButton_2_clicked()
{
    
}

void MainWindow::on_LegButton_3_clicked()
{
    
}

void MainWindow::on_LegButton_4_clicked()
{
    
}

void MainWindow::on_ArmButton_1_clicked()
{
    
}

void MainWindow::on_ArmButton_2_clicked()
{
    
}

void MainWindow::on_ArmButton_3_clicked()
{
    
}

void MainWindow::on_ArmButton_4_clicked()
{
    
}

void MainWindow::on_ArmUpButton_clicked()
{
    
}

void MainWindow::on_ArmDownButton_clicked()
{
    
}

void MainWindow::on_ArmLeftButton_clicked()
{
    
}

void MainWindow::on_ArmRightButton_clicked()
{
    
}

void MainWindow::on_AutoButton_clicked()
{
    
}

void MainWindow::on_ControlButton_clicked()
{
    
}

void MainWindow::on_InverseMoveButton_clicked()
{
    
}

void MainWindow::on_AllJointButton_clicked()
{
    
}

void MainWindow::on_MultiJointButton_clicked()
{
    
}

void MainWindow::on_ClearButton_clicked()
{
    
}

void MainWindow::on_OffSetButton_clicked()
{
    
}

void MainWindow::on_StatusButton_clicked()
{
    ui.JointSet_1->display(qnode.Joint_Set[0]);
    ui.JointSet_2->display(qnode.Joint_Set[1]);
    ui.JointSet_3->display(qnode.Joint_Set[2]);
//    ui.JointSet_4->display(qnode.Joint_Set[3]);
//    ui.JointSet_5->display(qnode.Joint_Set[4]);
//    ui.JointSet_6->display(qnode.Joint_Set[5]);
//    ui.JointSet_7->display(qnode.Joint_Set[6]);
//    ui.JointSet_8->display(qnode.Joint_Set[7]);
//    ui.JointSet_9->display(qnode.Joint_Set[8]);
//    ui.JointSet_10->display(qnode.Joint_Set[9]);
//    ui.JointSet_11->display(qnode.Joint_Set[10]);
    ui.JointSet_12->display(qnode.Joint_Set[11]);
    
    usleep(PKT_RTN_DELAY_US);
    
    ui.Velocity_1->display(qnode.Velocity[0]);
    ui.Velocity_2->display(qnode.Velocity[1]);
    ui.Velocity_3->display(qnode.Velocity[2]);
//    ui.Velocity_4->display(qnode.Velocity[3]);
//    ui.Velocity_5->display(qnode.Velocity[4]);
//    ui.Velocity_6->display(qnode.Velocity[5]);
//    ui.Velocity_7->display(qnode.Velocity[6]);
//    ui.Velocity_8->display(qnode.Velocity[7]);
//    ui.Velocity_9->display(qnode.Velocity[8]);
//    ui.Velocity_10->display(qnode.Velocity[9]);
//    ui.Velocity_11->display(qnode.Velocity[10]);
    ui.Velocity_12->display(qnode.Velocity[11]);
    usleep(PKT_RTN_DELAY_US);
    

    ui.OffSet_1->display(qnode.Off_Set[0]);
    ui.OffSet_2->display(qnode.Off_Set[1]);
    ui.OffSet_3->display(qnode.Off_Set[2]);
//    ui.OffSet_4->display(qnode.Off_Set[3]);
//    ui.OffSet_5->display(qnode.Off_Set[4]);
//    ui.OffSet_6->display(qnode.Off_Set[5]);
//    ui.OffSet_7->display(qnode.Off_Set[6]);
//    ui.OffSet_8->display(qnode.Off_Set[7]);
//    ui.OffSet_9->display(qnode.Off_Set[8]);
//    ui.OffSet_10->display(qnode.Off_Set[9]);
//    ui.OffSet_11->display(qnode.Off_Set[10]);
    ui.OffSet_12->display(qnode.Off_Set[11]);

    usleep(PKT_RTN_DELAY_US);

    ui.Position_1->display(qnode.Position[0]);
    ui.Position_2->display(qnode.Position[1]);
    ui.Position_3->display(qnode.Position[2]);
//    ui.Position_4->display(qnode.Position[3]);
//    ui.Position_5->display(qnode.Position[4]);
//    ui.Position_6->display(qnode.Position[5]);
//    ui.Position_7->display(qnode.Position[6]);
//    ui.Position_8->display(qnode.Position[7]);
//    ui.Position_9->display(qnode.Position[8]);
//    ui.Position_10->display(qnode.Position[9]);
//    ui.Position_11->display(qnode.Position[10]);
    ui.Position_12->display(qnode.Position[11]);

    usleep(PKT_RTN_DELAY_US);
    
    ui.Current_1->setValue(qnode.Current[0] / 6.3 * 100);
    ui.Current_2->setValue(qnode.Current[1] / 6.3 * 100);
    ui.Current_3->setValue(qnode.Current[2] / 6.3 * 100);
//    ui.Current_4->setValue(qnode.Current[3] / 6.3 * 100);
//    ui.Current_6->setValue(qnode.Current[5] / 5.2 * 100);
//    ui.Current_7->setValue(qnode.Current[6] / 5.2 * 100);
//    ui.Current_8->setValue(qnode.Current[7] / 5.2 * 100);
//    ui.Current_5->setValue(qnode.Temperature[0] / 80 * 100);
//    ui.Current_9->setValue(qnode.Temperature[1] / 80 * 100);
//    ui.Current_10->setValue(qnode.Temperature[2] / 80 * 100);
//    ui.Current_11->setValue(qnode.Temperature[3] / 80 * 100);
    ui.Current_12->setValue(qnode.Temperature[4] / 80 * 100);

}


}


