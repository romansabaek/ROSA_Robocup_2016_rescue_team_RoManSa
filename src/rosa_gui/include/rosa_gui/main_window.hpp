/**
 * @file /include/rosa_gui/main_window.hpp
 *
 * @brief Qt based gui for rosa_gui.
 *
 * @date November 2010
 **/
#ifndef rosa_gui_MAIN_WINDOW_H
#define rosa_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QPainter>
#include <QGraphicsItem>
#include <QDebug>
#include <math.h>
#include <QtGui>
#include <QMessageBox>
#include <iostream>

#include "ui_main_window.h"
#include "qnode.hpp"
#include "ros/ros.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rosa_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

private Q_SLOTS:
    //void on_listView_activated(const QModelIndex &index);

    void on_InitializeButton_clicked();
    
    void on_SafeEndButton_clicked();
    
    void on_EmergencyButton_clicked();
    
    void on_LegButton_1_clicked();
    
    void on_LegButton_2_clicked();
    
    void on_LegButton_3_clicked();
    
    void on_LegButton_4_clicked();
    
    void on_ArmButton_1_clicked();
    
    void on_ArmButton_2_clicked();
    
    void on_ArmButton_3_clicked();
    
    void on_ArmButton_4_clicked();
    
    void on_ArmUpButton_clicked();
    
    void on_ArmDownButton_clicked();
    
    void on_ArmLeftButton_clicked();
    
    void on_ArmRightButton_clicked();
    
    void on_AutoButton_clicked();
    
    void on_ControlButton_clicked();
    
    void on_InverseMoveButton_clicked();
    
    void on_AllJointButton_clicked();
    
    void on_MultiJointButton_clicked();
    
    void on_ClearButton_clicked();
    
    void on_OffSetButton_clicked();

    void on_StatusButton_clicked();

private:

        Ui::MainWindow ui;
        QGraphicsScene *scene;
        QGraphicsEllipseItem *ellipse;
    	QGraphicsRectItem *rectangle;
        QGraphicsLineItem *line[9];
    	QPen pen_ora, pen_yel,pen_blk;
	QNode qnode;
};

}  // namespace rosa_gui

#endif // rosa_gui_MAIN_WINDOW_H
