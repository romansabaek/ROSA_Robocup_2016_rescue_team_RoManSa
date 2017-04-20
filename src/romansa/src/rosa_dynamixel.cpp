#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <getopt.h>
#include <fcntl.h>
#include <math.h>
#include "../include/romansa/dynamixel.h"
#include "../include/romansa/serialport.h"
#include "../include/romansa/dynamixel_function.h"
#include "romansa/rosa_dynamixel_msg.h"
#include "../include/rosa_msg/command.h"
#include "../include/rosa_msg/readdata.h"
#include "../include/rosa_msg/kinematics.h"

// Control table address
#define P_CW_ANGLE_LIMIT         6
#define P_CCW_ANGLE_LIMIT        8
#define P_TORQUE_ENABLE         24
#define P_GOAL_POSITION		    30
#define P_GOAL_SPEED		    32
#define P_PRESENT_POSITION   	36
#define P_PRESENT_SPEED		    38
#define P_MOVING                46
#define P_PRESENT_JOINT         8
#define P_PRESENT_OFFSET        20
#define P_PRESENT_CURRENT       68
#define P_PRESENT_TEMPERRATURE  43


#define NUM_ACTUATOR		     4
#define CONTROL_PERIOD		10000 // usec (Large value is more slow)
#define PKT_RTN_DELAY_US	5000




using namespace std;

void RunDxCallback(const romansa::rosa_dynamixel_msg::ConstPtr& msg);
void readmotorcallback(const rosa_msg::readdata::ConstPtr& msg);
void AutoKinematicsRosaArm(const rosa_msg::kinematics::ConstPtr& msg);
void Syncwrite_Packet();
void inverse_matrix(double &a11,double &a12,double &a13,
                    double &a21,double &a22,double &a23,
                    double &a31,double &a32,double &a33 );

void inverse_arm(double &theta1,double &theta2,double &theta3);
void inverse_play(double Desired_X,double x,double y, double Desired_Y);
void sum_kinematic();
void Read_theta();
void Syncwrite_speed_Packet();

ros::Publisher gui_read_pub;


int P_posi[] ={0,};
int P_speed[] ={0,};
int P_JOINT[]  ={0,};
int P_OFFSET[] ={0,};
int P_CURRENT[] = {0,};
int P_TEMPERATURE[]= {0,};

//auto posi//
double Desired_X =0;
double Desired_Y =0;
double Desired_Z =0;
double Pre_X =0;
double Pre_Y =0;
double Pre_Z =0;
double P_theta[3] ={0,};
double theta[3];
double theta_dot[3]={0,};
double result[3]={0,};
double r;
double Vx,Vy,V_x,V_y,Wz;
double determinent;
double Jacobian[3][3];
double Inv_Jacobian[3][3];
double final_angle[3] = {0,};

//parameter//
double link1 = 0.3;
double link2 = 0.44;
double link3 = 0.23;
int dt=5;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSA_dynamixel");
    ros::NodeHandle nh;

    ros::Subscriber rosa_dynamixel_sub = nh.subscribe<romansa::rosa_dynamixel_msg>("rosa_dynamixel", 100, RunDxCallback);

    ros::Subscriber rosa_motor_read_sub = nh.subscribe<rosa_msg::readdata>("read_data",100,readmotorcallback);
    ros::Subscriber rosa_autokinematics_sub = nh.subscribe<rosa_msg::kinematics>("rosa_kinematics",100,AutoKinematicsRosaArm);


    gui_read_pub = nh.advertise<rosa_msg::readdata>("read_data",100);




//    rosa_gui::readdata read;

    if (MOT_InitUART() == 0)
    {
        printf("ERROR : InitUART\n");
        return 0;
    }
    sleep(1);

    ros::spin();

    return 0;
}

////make packet////

void Syncwrite_POSI_Packet()
{
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0,P_GOAL_POSITION);
    dxl_set_txpacket_parameter(1, 2);
    dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);
    dxl_txrx_packet();

}

void Syncwrite_speed_Packet()
{
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0,P_GOAL_SPEED);
    dxl_set_txpacket_parameter(1, 2);
    dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);
    dxl_txrx_packet();
}
///auto mode kinematics///
void Read_theta()
{

    for(int i=0;i<3;i++)
    P_theta[i]=theta[i]; //present position save

    ///depend base cordination///
    theta[0]=theta[0] - 1024; // 5 , 6 joint

    if(theta[1] <= 2048)
    {
        theta[1] = 2048 - theta[1];
    }
    else
    {
        theta[1] = theta[1] - 2048;
        theta[1] = -theta[1];

    }


    if(theta[2] >= 2048)
    {
        theta[2] = theta[2] - 2048;
    }
    else
    {
        theta[2] = 2048 - theta[2];
        theta[2] = -theta[2];
    }

    //motor 4096 -> degree 360
    for(int i=0;i<3;i++)
        theta[i] = (int)theta[i]*360/4096;
    //degree -> radian
    for(int i=0;i<3;i++)
        theta[i] = (int)theta[i]*M_PI/180;

}

void inverse_arm(double &theta1,double &theta2,double &theta3) //jacobian matrix
{


        Jacobian[0][0] =  -link2*sin(theta1 + theta2) - link1*sin(theta1) - link3*sin(theta1 + theta2 + theta3);

        Jacobian[0][1] =  -link2*sin(theta1 + theta2) - link3*sin(theta1 + theta2 + theta3);

        Jacobian[0][2] = - link3*sin(theta1 + theta2 + theta3);

        Jacobian[1][0] = link2*cos(theta1 + theta2) + link1*cos(theta1) + link3*cos(theta1 + theta2 + theta3);

        Jacobian[1][1] = link2*cos(theta1 + theta2) + link3*cos(theta1 + theta2 + theta3);

        Jacobian[1][2] = link3*cos(theta1 + theta2 + theta3);

        Jacobian[2][0] = 1;
        Jacobian[2][1] = 1;
        Jacobian[2][2] = 1;

        printf("jacobialn Matrix\n");
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                printf("%f  ",Jacobian[i][j]);
            }printf("\n");
        }printf("\n");

        inverse_matrix(Jacobian[0][0],Jacobian[0][1],Jacobian[0][2],Jacobian[1][0],Jacobian[1][1],Jacobian[1][2],Jacobian[2][0],Jacobian[2][1],Jacobian[2][2]);

}


void inverse_matrix(double &a11,double &a12,double &a13,
                    double &a21,double &a22,double &a23,
                    double &a31,double &a32,double &a33 )
{
    determinent = (a11*a22*a33 + a12*a23*a31 + a13*a21*a32 + a13*a21*a32)-(a31*a22*a13 + a32*a23*a11 + a33*a21*a12);

    if(determinent == 0)
    {
        printf("determinent = 0 \n");

    }
    else       //adj A
    {
        Inv_Jacobian[0][0] = (a22*a33-a23*a32)/determinent;


        Inv_Jacobian[0][1] = -(a12*a33-a13*a32)/determinent;

        Inv_Jacobian[0][2] = (a12*a23-a13*a22)/determinent;

        Inv_Jacobian[1][0] = -(a21*a33-a23*a31)/determinent;

        Inv_Jacobian[1][1] = (a11*a33-a13*a31)/determinent;

        Inv_Jacobian[1][2] = -(a11*a23-a13*a21)/determinent;

        Inv_Jacobian[2][0] = (a21*a32-a22*a31)/determinent;

        Inv_Jacobian[2][1] = -(a11*a32-a12*a31)/determinent;

        Inv_Jacobian[2][2] = (a11*a22-a12*a21)/determinent;

        printf("inv_jacobialn\n");
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                printf("%f  ",Inv_Jacobian[i][j]);
            }printf("\n");
        }
    }inverse_play(Desired_X,Pre_X,Pre_Y,Desired_Y);
}



void inverse_play(double Desired_X,double x,double y, double Desired_Y)
{

    Vx=V_x;
    Vy=V_y;

    //////////////////////////////////Wz///////////////////////////////////

//    if(x<0) P_W_Theta= 180+(atan(y/x)*180/M_PI);
//    else if(x>0) P_W_Theta= atan(y/x)*180/M_PI;
//    if(x2<0) A_W_Theta= 180+(atan(y2/x2)*180/M_PI);
//    else if(x2>0) A_W_Theta= atan(y2/x2)*180/M_PI;
    Wz= 0; //((A_W_Theta-P_W_Theta)/time)*M_PI/180;


    //////////////////////////////////Wz///////////////////////////////////

    theta_dot[0]= Inv_Jacobian[0][0]*Vx + Inv_Jacobian[0][1]*Vy + Inv_Jacobian[0][2] *Wz;
    theta_dot[1]= Inv_Jacobian[1][0]*Vx + Inv_Jacobian[1][1]*Vy + Inv_Jacobian[1][2] *Wz;
    theta_dot[2]= Inv_Jacobian[2][0]*Vx + Inv_Jacobian[2][1]*Vy + Inv_Jacobian[2][2] *Wz;


    //rad/s -> rpm//      //0~1023 = 0~ 117rpm//   //even number change//
    theta_dot[0]= theta_dot[0]*30690/(117*M_PI);
    theta_dot[1]= theta_dot[1]*30690/(117*M_PI);
    theta_dot[2]= theta_dot[2]*30690/(117*M_PI);


    theta[0] = P_theta[0] + theta_dot[0] * dt;
    theta[1] = P_theta[1] + theta_dot[1] * dt;
    theta[2] = P_theta[2] + theta_dot[2] * dt;

    for(int i=0;i<3;i++)
    {
        if(theta[i]>4096) theta[i] = theta[i]-4096;
        else if(theta[i]<0) theta[i] = theta[i]+4096;
        else theta[i] = theta[i];
    }

    for(int i=0;i<3;i++)
    {
        result[i]=theta[i];
    }

    if((0<result[1])&&(result[1]<2048))
    {
        result[1] = (2048-result[1])*2 + 10000;
    }
    else if((2048<result[1])&&(result[1]<3072))
    {
        result[1] = - (result[1]-2048)*2 + 10000;
    }
    printf("%f,  %f,  %f\n",result[0],result[1],result[2]);

    ///final angle print///
//    if(result[0]<2048)
//    {
//        final_angle[0] = -(result[0]-2048)*360/4096
//    }
}



void AutoKinematicsRosaArm(const rosa_msg::kinematics::ConstPtr& msg)
{
    rosa_msg::readdata kinematics;
    float test;

    Desired_X = msg->ROSA_KINEMATICS_DESIRED_POSI[0];
    Desired_Y = msg->ROSA_KINEMATICS_DESIRED_POSI[1];
    Desired_Z = msg->ROSA_KINEMATICS_DESIRED_POSI[2];

    printf("%f %f %f\n", Desired_X,Desired_Y,Desired_Z);
    theta[0] = msg->theta[0];
    theta[1] = msg->theta[1];
    theta[2] = msg->theta[2];
    printf("%f,  %f,  %f\n",theta[0],theta[1],theta[2]);
//    scanf("%fl",&test);

    while(ros::ok())
    {
        //scanf("%fl",&test);
        printf("%f,  %f,  %f\n",theta[0],theta[1],theta[2]);
        Read_theta();
        //forward kinematics//
        printf("after read theta %f,  %f,  %f\n",theta[0],theta[1],theta[2]);
        Pre_X = link1*cos(theta[0]) + link2*cos(theta[0]+theta[1]) + link3*cos(theta[0]+theta[1]+theta[2]);
        Pre_Y = link1*sin(theta[0]) + link2*sin(theta[0]+theta[1]) + link3*sin(theta[0]+theta[1]+theta[2]);

        r = sqrt((Desired_X-Pre_X)*(Desired_X-Pre_X) + (Desired_Y-Pre_Y)*(Desired_Y-Pre_Y));

        printf("x,y,r = %f %f %f\n",Pre_X,Pre_Y,r);
        if(r<0.01)
        {
            printf("success kinematics!\n");
            break;
        }
        ////////////////////////present x,y data//////////////////////////////////
        V_x= (Desired_X-Pre_X)/dt;
        V_y= -(Desired_Y-Pre_Y)/dt;

        inverse_arm(theta[0],theta[1],theta[2]);
        printf("after inverse arm %f,  %f,  %f\n",theta[0],theta[1],theta[2]);
    }


}




void readmotorcallback(const rosa_msg::readdata::ConstPtr& msg)
{
    rosa_msg::readdata motor_read;

    switch(msg->arm)
    {
        case rosa_msg::readdata::ROSA_DX_READ_DATA :

        for( int i=0; i<msg->id.size(); i++ )
        {
            P_posi[i] = dxl_read_word(msg->id[i],P_PRESENT_POSITION);
            usleep(PKT_RTN_DELAY_US);

            P_speed[i] = dxl_read_word(msg->id[i],P_PRESENT_SPEED);
            usleep(PKT_RTN_DELAY_US);

            P_JOINT[i] = dxl_read_word(msg->id[i],P_PRESENT_JOINT);
            usleep(PKT_RTN_DELAY_US);

            P_OFFSET[i] = dxl_read_word(msg->id[i],P_PRESENT_OFFSET);
            usleep(PKT_RTN_DELAY_US);

            P_CURRENT[i] = dxl_read_word(msg->id[i],P_PRESENT_CURRENT);
            usleep(PKT_RTN_DELAY_US);

            P_TEMPERATURE[i] = dxl_read_word(msg->id[i],P_PRESENT_TEMPERRATURE);
            usleep(PKT_RTN_DELAY_US);
        }
            //DATA SAVE//
            for(int i=0; i<12; i++)
            {
                motor_read.DX_POSI.push_back(P_posi[i]);
                usleep(PKT_RTN_DELAY_US);
            }
            for(int i=0; i<12; i++)
            {
                motor_read.DX_SPEED.push_back(P_speed[i]);
                usleep(PKT_RTN_DELAY_US);
            }
            for(int i=0; i<12; i++)
            {
                motor_read.DX_JOINT.push_back(P_JOINT[i]);
                usleep(PKT_RTN_DELAY_US);
            }
            for(int i=0; i<12; i++)
            {
                motor_read.DX_OFFSET.push_back(P_OFFSET[i]);
                usleep(PKT_RTN_DELAY_US);
            }
            for(int i=0; i<12; i++)
            {
                motor_read.DX_CURRENT.push_back(P_CURRENT[i]);
                usleep(PKT_RTN_DELAY_US);
            }
            for(int i=0; i<12; i++)
            {
                motor_read.DX_TEMPERATURE.push_back(P_TEMPERATURE[i]);
                usleep(PKT_RTN_DELAY_US);
            }

            gui_read_pub.publish(motor_read);



            motor_read.DX_POSI.clear();
            motor_read.DX_SPEED.clear();
            motor_read.DX_JOINT.clear();
            motor_read.DX_OFFSET.clear();
            motor_read.DX_CURRENT.clear();
            motor_read.DX_TEMPERATURE.clear();

        for(int i=0; i<12; i++)
        {
             ROS_INFO(" speed %d : %d\n",i+1,P_speed[i]);
        }

        for(int i=0; i<12; i++)
        {
             ROS_INFO("joint %d : %d\n",i+1,P_JOINT[i]);
        }
        for(int i=0; i<12; i++)
        {
             ROS_INFO("offset %d : %d\n",i+1,P_OFFSET[i]);
        }
        for(int i=0; i<12; i++)
        {
             ROS_INFO("position %d : %d\n",i+1,P_posi[i]);
        }
        for(int i=0; i<12; i++)
        {
             ROS_INFO("temp %d : %d\n",i+1,P_TEMPERATURE[i]);
        }
        break;

    }
}







void RunDxCallback(const romansa::rosa_dynamixel_msg::ConstPtr& msg)
{

    switch(msg->mode)
    {

        case romansa::rosa_dynamixel_msg::ROSA_DX_GOAL_POSITION :
            for( int i=0; i<msg->id.size(); i++ )
            {
                dxl_set_txpacket_parameter(2+3*i, msg->id[i]);
                dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(P_posi[i]+ msg->Goal_position[i]));
                dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(P_posi[i]+ msg->Goal_position[i]));
                Syncwrite_POSI_Packet();
                usleep(CONTROL_PERIOD);
            }

            ROS_INFO("2048 1024 2048 ok\n");
        break;



        case romansa::rosa_dynamixel_msg::ROSA_DX_GOAL_SPEED :
            for( int i=0; i<msg->id.size(); i++ )
            {
//                dxl_set_txpacket_parameter(2+3*i, msg->id[i]);
//                dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(msg->Moving_speed[i]));
//                dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(msg->Moving_speed[i]));
//                ROS_INFO("%d",msg->Moving_speed[i]);
//                Syncwrite_speed_Packet();
                dxl_write_word(msg->id[i],P_GOAL_SPEED,msg->Moving_speed[i]);
                usleep(CONTROL_PERIOD);
            }

            ROS_INFO("speed setting ok\n");
        break;


      ////////////////////////////////////////////////////////////////////////////////
        case romansa::rosa_dynamixel_msg::ROSA_DX_PART_UP_RIGHT :

            for( int i=0; i<msg->id.size(); i++ )
            {
                P_posi[i] = dxl_read_word(msg->id[i],P_GOAL_POSITION);
                usleep(PKT_RTN_DELAY_US);
            }

            for( int i=0; i<msg->id.size(); i++ )
            {            
                dxl_set_txpacket_parameter(2+3*i, msg->id[i]);
                dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(P_posi[i]+ msg->Goal_position[i]));
                dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(P_posi[i]+ msg->Goal_position[i]));
                ROS_INFO("%d",msg->id[i]);
                Syncwrite_POSI_Packet();
                usleep(CONTROL_PERIOD);
            }

            ROS_INFO("part control UP ok\n");
        break;

        case romansa::rosa_dynamixel_msg::ROSA_DX_PART_DOWN_LEFT :

            for( int i=0; i<msg->id.size(); i++ )
            {
                P_posi[i] = dxl_read_word(msg->id[i],P_GOAL_POSITION);
                usleep(PKT_RTN_DELAY_US);
            }

            for( int i=0; i<msg->id.size(); i++ )
            {
                dxl_set_txpacket_parameter(2+3*i, msg->id[i]);
                dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(P_posi[i] - msg->Goal_position[i]));
                dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(P_posi[i] - msg->Goal_position[i]));
                ROS_INFO("%d",msg->id[i]);
                Syncwrite_POSI_Packet();
                usleep(CONTROL_PERIOD);
            }
            ROS_INFO("part control DOWN ok\n");

         break;

         case romansa::rosa_dynamixel_msg::DX_KINEMATICS_MOVE :

            ROS_INFO("move ok");
            for( int i=0; i<3; i++ )
            {
                if(i==2) //id 7
                {
                    dxl_set_txpacket_parameter(2+3*i, msg->id[i]);
                    dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(result[1]));
                    dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(result[1]));
                }
                else //id 5,6
                {
                    dxl_set_txpacket_parameter(2+3*i, msg->id[i]);
                    dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(result[0]));
                    dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(result[0]));
                }
            }
            dxl_set_txpacket_parameter(2+3*3, msg->id[3]);
            dxl_set_txpacket_parameter(2+3*3+1, dxl_get_lowbyte(result[2]));
            dxl_set_txpacket_parameter(2+3*3+2, dxl_get_highbyte(result[2]));

            Syncwrite_POSI_Packet();
            usleep(CONTROL_PERIOD);
         break;

    }
}

















