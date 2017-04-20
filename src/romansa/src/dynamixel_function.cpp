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
#include "../include/romansa/dynamixel_function.h"
#include "../include/romansa/dynamixel.h"
#include "../include/romansa/serialport.h"

// Control table address
#define P_CW_ANGLE_LIMIT         6
#define P_CCW_ANGLE_LIMIT        8
#define P_TORQUE_ENABLE         24
#define P_GOAL_POSITION		30
#define P_GOAL_SPEED		32
#define P_PRESENT_POSITION	36
#define P_PRESENT_SPEED		38
#define P_MOVING                46

//using namespace std;
//using namespace DXL_PRO;
//Dynamixel DXL("/dev/ttyUSB0"); // Port Name Setting

int result = COMM_TXFAIL, error = 0;
int deviceIndex =0;


// Value setting
#define VAL_BAUDNUM        	1	// 115,200
#define PKT_RTN_DELAY_US	5000

int CW[13] = {0,
           0,    0,    0,    0,    4095,
           0,    0,    0,    4095,    4095,
           4095,    4095
         };
int CCW[13] = {0,
        4095,    4095, 4095, 4095, 4095,
        4095,    4095, 4095, 4095, 4095,
        4095,    4095
         };		// Center = 2048



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
//////////////
//// uart ////
//////////////

int MOT_InitUART()
{
    // Serial Port Connection
    if( dxl_initialize(deviceIndex, VAL_BAUDNUM) == 0 )
    {
        printf("Failed to open the PORT\n");
        printf("Failed to change the baudrate!\n");
        return 0;
    }
    else
        printf("Succeed to open the PORT\n");
        printf("Succeed to change the baudrate!\n");

    return 1;
}

void MOT_CloseUART()
{
    //DXL.Disconnect();	// Close PORT
    dxl_hal_close();
}

/////////////////////
////torque on off////
/////////////////////

void MOT_InitDXLTorque()
{
    // ALL MOTOR TORQUE :: ON
    //ROS_INFO("TORQUE");
    for(int ID=1; ID<6; ID++)
    {
        dxl_write_byte(ID, 24, 1);
        usleep(5000);
    }
}

void MOT_DisposeDXLTorque()
{
    // ALL MOTOR TORQUE :: OFF
    for(int ID=1; ID<13; ID++)
    {
        dxl_write_byte(ID, P_TORQUE_ENABLE,0);
        usleep(PKT_RTN_DELAY_US);
    }
}



//// init mode ////

void MOT_InitDXLMode()
{
    int ID;
    for(int ID=1; ID<13; ID++)
    {
        dxl_write_word(ID , P_CW_ANGLE_LIMIT, CW[ID]);
        usleep(PKT_RTN_DELAY_US);
        dxl_write_word(ID , P_CCW_ANGLE_LIMIT, CCW[ID]);
        usleep(PKT_RTN_DELAY_US);
    }
}


void MOT_Write(int id, int Portnum, int Data)
{
    dxl_write_word(id, Portnum, Data);
    //usleep(PKT_RTN_DELAY_US);
}


void MOT_Syncwrite(int *id, int NUM_ACTUATOR, int *GoalPos)
{
    dxl_set_txpacket_id(BROADCAST_ID);
    dxl_set_txpacket_instruction(INST_SYNC_WRITE);
    dxl_set_txpacket_parameter(0, P_GOAL_POSITION);
    dxl_set_txpacket_parameter(1, 2);

    for( int i=0; i<NUM_ACTUATOR; i++ )
    {
        dxl_set_txpacket_parameter(2+3*i, id[i]);
        dxl_set_txpacket_parameter(2+3*i+1, dxl_get_lowbyte(GoalPos[i]));
        dxl_set_txpacket_parameter(2+3*i+2, dxl_get_highbyte(GoalPos[i]));
    }
    dxl_set_txpacket_length((2+1)*NUM_ACTUATOR+4);
    dxl_txrx_packet();
}


int MOT_Read(int id, int Portnum)
{
    int temp;

    result = dxl_read_word(id, Portnum);
    if( result == COMM_RXSUCCESS )
    {
        printf("success\n");
        return temp;
    }
}

// Print communication result
void PrintCommStatus(int CommStatus)
{
    switch(CommStatus)
    {
    case COMM_TXFAIL:
        printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
        break;

    case COMM_TXERROR:
        printf("COMM_TXERROR: Incorrect instruction packet!\n");
        break;

    case COMM_RXFAIL:
        printf("COMM_RXFAIL: Failed get status packet from device!\n");
        break;

    case COMM_RXWAITING:
        printf("COMM_RXWAITING: Now recieving status packet!\n");
        break;

    case COMM_RXTIMEOUT:
        printf("COMM_RXTIMEOUT: There is no status packet!\n");
        break;

    case COMM_RXCORRUPT:
        printf("COMM_RXCORRUPT: Incorrect status packet!\n");
        break;

    default:
        printf("This is unknown error code!\n");
        break;
    }
}

void PrintErrorCode(int ErrorCode)
{
    if(ErrorCode & ERRBIT_VOLTAGE)
        printf("Input voltage error!\n");

    if(ErrorCode & ERRBIT_ANGLE)
        printf("Angle limit error!\n");

    if(ErrorCode & ERRBIT_OVERHEAT)
        printf("Overheat error!\n");

    if(ErrorCode & ERRBIT_RANGE)
        printf("Out of range error!\n");

    if(ErrorCode & ERRBIT_CHECKSUM)
        printf("Checksum error!\n");

    if(ErrorCode & ERRBIT_OVERLOAD)
        printf("Overload error!\n");

    if(ErrorCode & ERRBIT_INSTRUCTION)
        printf("Instruction code error!\n");
}
