#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <getopt.h>
#include <fcntl.h>
#include <iostream>
#include <vector>
#include <sstream>
#include "romansa/rosa_dynamixel_msg.h"
#include "../include/rosa_msg/command.h"
#include "../include/rosa_msg/readdata.h"
#include "../include/rosa_msg/kinematics.h"
//control//
#define F_Leg_up         'Q'
#define B_Leg_up         'W'
#define F_Leg_down       'A'
#define B_Leg_down       'S'
#define W_Leg_up         'E'
#define W_Leg_down       'D'

#define arm_M1_right     'R'
#define arm_M1_left      'F'
#define arm_link1_up     'T'
#define arm_link1_down   'G'
#define arm_link2_up     'Y'
#define arm_link2_down   'H'
#define arm_M5_right     'U'
#define arm_M5_left      'J'
#define arm_link3_up     'I'
#define arm_link3_down   'K'
#define arm_M7_right     'O'
#define arm_M7_left      'L'
#define gripper_on       'P'
#define gripper_off      ';'

//Leg key//
#define fornt_left_leg   '1'
#define fornt_right_leg   '2'
#define rear_left_leg   '3'
#define rear_right_leg   '4'

//read data//
#define read_data       'Z'

int _getch();
void CommandLegControlcallback(const rosa_msg::command::ConstPtr& msg);
void Front_Leg_UP(romansa::rosa_dynamixel_msg Front_leg_up);
void Front_Leg_DOWN(romansa::rosa_dynamixel_msg Front_leg_down);
void Rear_Leg_UP(romansa::rosa_dynamixel_msg Rear_leg_up);
void Rear_Leg_DOWN(romansa::rosa_dynamixel_msg Rear_leg_down);
void Whole_Leg_UP(romansa::rosa_dynamixel_msg whole_leg_up);
void Whole_Leg_DOWN(romansa::rosa_dynamixel_msg whole_leg_down);
void DX_SPEED_SETTING(romansa::rosa_dynamixel_msg speed);
void DX_READ_MOTOR(rosa_msg::readdata readdata);
void test_kinematics(rosa_msg::kinematics kinematics);
void kinematics_move(romansa::rosa_dynamixel_msg move);
void init_kinematics(romansa::rosa_dynamixel_msg init_kinematic);

ros::Publisher rosa_kinematics_pub;
ros::Publisher rosa_dynamixel_pub;
ros::Publisher rosa_command_pub;
ros::Publisher rosa_gui_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROSA_dynamixel_pub");
    ros::NodeHandle nh;
    
    rosa_dynamixel_pub = nh.advertise<romansa::rosa_dynamixel_msg>("rosa_dynamixel", 100);

    rosa_command_pub =nh.advertise<rosa_msg::command>("command",100);
    rosa_gui_pub =nh.advertise<rosa_msg::readdata>("read_data",100);
    rosa_kinematics_pub=nh.advertise<rosa_msg::kinematics>("rosa_kinematics",100);

    ros::Rate loop_rate(10);
   
    //////////////////////////////////////////////////////
    romansa::rosa_dynamixel_msg init_kinematic;
    init_kinematic.mode = romansa::rosa_dynamixel_msg::ROSA_DX_GOAL_POSITION;

    romansa::rosa_dynamixel_msg DX_GOAL_SPEED;
    DX_GOAL_SPEED.mode = romansa::rosa_dynamixel_msg::ROSA_DX_GOAL_SPEED;


    romansa::rosa_dynamixel_msg DX_PART_U;
    DX_PART_U.mode = romansa::rosa_dynamixel_msg::ROSA_DX_PART_UP_RIGHT;

    romansa::rosa_dynamixel_msg DX_PART_D;
    DX_PART_D.mode = romansa::rosa_dynamixel_msg::ROSA_DX_PART_DOWN_LEFT;

    romansa::rosa_dynamixel_msg KINEMATICS_MOVE;
    KINEMATICS_MOVE.mode = romansa::rosa_dynamixel_msg::DX_KINEMATICS_MOVE;

    rosa_msg::readdata read;
    read.arm = rosa_msg::readdata::ROSA_DX_READ_DATA;

    rosa_msg::kinematics kinematics;
    rosa_msg::command command;

    while (ros::ok())
    {	
		
        switch(_getch())
        {
            case '0':
            ROS_INFO("auto mode kinematics!\n");
            test_kinematics(kinematics);
            break;

            case '1':
            ROS_INFO("AUTO MOVE\n");
            kinematics_move(KINEMATICS_MOVE);
            break;

            case '2':
            ROS_INFO("GOAL SPEED\n");
            DX_SPEED_SETTING(DX_GOAL_SPEED);
            break;

            case '3':
            ROS_INFO("initkinematics\n");
            init_kinematics(init_kinematic);
            break;

            /////read data//////
            case read_data: case 'z':
            ROS_INFO("read data\n");
            DX_READ_MOTOR(read);
            break;


            //////////////leg control////////////////////////////////////////////
            case F_Leg_up: case 'q':
            ROS_INFO("F_Leg_up\n");
            Front_Leg_UP(DX_PART_U);
            break;

            case F_Leg_down: case 'a':
            ROS_INFO("F_Leg_down\n");
            Front_Leg_DOWN(DX_PART_D);
            break;

            case B_Leg_up: case 'w':
            ROS_INFO("B_Leg_up\n");
            Rear_Leg_UP(DX_PART_U);
            break;

            case B_Leg_down: case 's':
            ROS_INFO("B_Leg_down\n");
            Rear_Leg_DOWN(DX_PART_D);
            break;

            case W_Leg_up: case 'e':
            ROS_INFO("W_Leg_up\n");
            Whole_Leg_UP(DX_PART_U);
            break;

            case W_Leg_down: case 'd':
            ROS_INFO("W_Leg_down\n");
            Whole_Leg_DOWN(DX_PART_D);
            break;

            ///////arm control//////////////////////////////////////////////////////////////////
            /////////////////////////////////////////////////////////////////////////////////////

            case arm_M1_right: case 'r':
            ROS_INFO("arm_M1_right");

            break;

            case arm_M1_left: case 'f':
            ROS_INFO("arm_M1_left");

            break;

            case arm_link1_up: case 't':
            ROS_INFO("arm_link1_up");

            break;

            case arm_link1_down: case 'g':
            ROS_INFO("arm_link1_down");

            break;

            case arm_link2_up: case 'y':
            ROS_INFO("arm_link2_up");

            break;

            case arm_link2_down: case 'h':
            ROS_INFO("arm_link2_down");

            break;

            case arm_M5_right: case 'u':
            ROS_INFO("arm_M5_right");

            break;

            case arm_M5_left: case 'j':
            ROS_INFO("arm_M5_left");

            break;

            case arm_link3_up: case 'i':
            ROS_INFO("arm_link3_up");

            break;

            case arm_link3_down: case 'k':
            ROS_INFO("arm_link3_down");

            break;

            case arm_M7_right: case 'o':
            ROS_INFO("arm_M7_right");

            break;

            case arm_M7_left: case 'l':
            ROS_INFO("arm_M7_left");

            break;

            case gripper_on: case 'p':
            ROS_INFO("gripper_on");

            break;

            case gripper_off:
            ROS_INFO("gripper_off");

            break;


        }
        loop_rate.sleep();
    }
    return 0;
}

int _getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}
//////test/////////
void DX_SPEED_SETTING(romansa::rosa_dynamixel_msg speed)
{
    speed.mode = romansa::rosa_dynamixel_msg::ROSA_DX_GOAL_SPEED;

    for(int id=1;id<5;id++)
    {
        speed.id.push_back(id);
        speed.Moving_speed.push_back(300);
    }
    for(int id=5;id<7;id++)
    {
        speed.id.push_back(id);
        speed.Moving_speed.push_back(50);
    }
    speed.id.push_back(7);
    speed.Moving_speed.push_back(100);
    for(int id=8;id<13;id++)
    {
        speed.id.push_back(id);
        speed.Moving_speed.push_back(50);
    }
    rosa_dynamixel_pub.publish(speed);
    speed.id.clear();
    speed.Moving_speed.clear();
}


///test kinematics pub//
void init_kinematics(romansa::rosa_dynamixel_msg init_kinematic)
{
    init_kinematic.mode = romansa::rosa_dynamixel_msg::ROSA_DX_GOAL_POSITION;

    init_kinematic.id.push_back(5);
    init_kinematic.id.push_back(6);
    init_kinematic.id.push_back(7);
    init_kinematic.id.push_back(9);

    init_kinematic.Goal_position.push_back(2048);
    init_kinematic.Goal_position.push_back(2048);
    init_kinematic.Goal_position.push_back(12048);
    init_kinematic.Goal_position.push_back(2048);

    rosa_dynamixel_pub.publish(init_kinematic);
    init_kinematic.id.clear();
    init_kinematic.Goal_position.clear();

}
void test_kinematics(rosa_msg::kinematics kinematics)
{

    kinematics.ROSA_KINEMATICS_DESIRED_POSI.push_back(-0.54); //x
    kinematics.ROSA_KINEMATICS_DESIRED_POSI.push_back(0.3);  //y
    kinematics.ROSA_KINEMATICS_DESIRED_POSI.push_back(0);  //z

    kinematics.theta.push_back(2048);
    kinematics.theta.push_back(1024);
    kinematics.theta.push_back(2048);

    rosa_kinematics_pub.publish(kinematics);
    kinematics.ROSA_KINEMATICS_DESIRED_POSI.clear();
    kinematics.theta.clear();
}

void kinematics_move(romansa::rosa_dynamixel_msg move)
{
    move.mode= romansa::rosa_dynamixel_msg::DX_KINEMATICS_MOVE;

    for(int i=5; i<8; i++)
    {
        move.id.push_back(i);
    }
    move.id.push_back(9);
    rosa_dynamixel_pub.publish(move);
    move.id.clear();
}

////DX_READ_MOTOR//////
void DX_READ_MOTOR(rosa_msg::readdata readdata)
{
    readdata.arm = rosa_msg::readdata::ROSA_DX_READ_DATA;

    for(int i=1; i<13; i++)
    {
        readdata.id.push_back(i);
    }

    rosa_gui_pub.publish(readdata);
    readdata.id.clear();
}




///////////////////////////////////////////////////////////
////leg control function//// +-300/////////////////////////
///////////////////////////////////////////////////////////

void Front_Leg_UP(romansa::rosa_dynamixel_msg Front_leg_up)
{
    Front_leg_up.mode = romansa::rosa_dynamixel_msg::ROSA_DX_PART_UP_RIGHT;

    for(int i =0; i<3;i++)
    {
        Front_leg_up.id.push_back(i);
    }
    for(int i=0; i<3; i++)
    {
        Front_leg_up.Goal_position.push_back(300);
    }
    rosa_dynamixel_pub.publish(Front_leg_up);
    Front_leg_up.id.clear();
    Front_leg_up.Goal_position.clear();
}

void Front_Leg_DOWN(romansa::rosa_dynamixel_msg Front_leg_down)
{
    Front_leg_down.mode = romansa::rosa_dynamixel_msg::ROSA_DX_PART_DOWN_LEFT;

    for(int i =0; i<3;i++)
    {
        Front_leg_down.id.push_back(i);
    }
    for(int i=0; i<3; i++)
    {
        Front_leg_down.Goal_position.push_back(300);
    }
    rosa_dynamixel_pub.publish(Front_leg_down);
    Front_leg_down.id.clear();
    Front_leg_down.Goal_position.clear();
}



void Rear_Leg_UP(romansa::rosa_dynamixel_msg Rear_leg_up)
{
    Rear_leg_up.mode = romansa::rosa_dynamixel_msg::ROSA_DX_PART_UP_RIGHT;

    for(int i =3; i<5;i++)
    {
        Rear_leg_up.id.push_back(i);
    }
    for(int i=3; i<5; i++)
    {
        Rear_leg_up.Goal_position.push_back(300);
    }
    rosa_dynamixel_pub.publish(Rear_leg_up);
    Rear_leg_up.id.clear();
    Rear_leg_up.Goal_position.clear();
}

void Rear_Leg_DOWN(romansa::rosa_dynamixel_msg Rear_leg_down)
{
    Rear_leg_down.mode = romansa::rosa_dynamixel_msg::ROSA_DX_PART_DOWN_LEFT;

    for(int i =3; i<5;i++)
    {
        Rear_leg_down.id.push_back(i);
    }
    for(int i=3; i<5; i++)
    {
        Rear_leg_down.Goal_position.push_back(300);
    }
    rosa_dynamixel_pub.publish(Rear_leg_down);
    Rear_leg_down.id.clear();
    Rear_leg_down.Goal_position.clear();
}

void Whole_Leg_UP(romansa::rosa_dynamixel_msg whole_leg_up)
{
    whole_leg_up.mode =romansa::rosa_dynamixel_msg::ROSA_DX_PART_UP_RIGHT;

    for(int i =1; i<5;i++)
    {
        whole_leg_up.id.push_back(i);
    }
    for(int i=1; i<5; i++)
    {
        whole_leg_up.Goal_position.push_back(300);
    }
    rosa_dynamixel_pub.publish(whole_leg_up);
    whole_leg_up.id.clear();
    whole_leg_up.Goal_position.clear();

}

void Whole_Leg_DOWN(romansa::rosa_dynamixel_msg whole_leg_down)
{
    whole_leg_down.mode =romansa::rosa_dynamixel_msg::ROSA_DX_PART_DOWN_LEFT;

    for(int i =1; i<5;i++)
    {
        whole_leg_down.id.push_back(i);
    }
    for(int i=1; i<5; i++)
    {
        whole_leg_down.Goal_position.push_back(300);
    }
    rosa_dynamixel_pub.publish(whole_leg_down);
    whole_leg_down.id.clear();
    whole_leg_down.Goal_position.clear();

}



/////////////////////////////end////////////////////////
























