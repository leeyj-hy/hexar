#include "serial.hpp"
#include "crcModbus.hpp"
#include <unistd.h>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "hexar_msgs/mod.h"
#include "hexar_msgs/pos.h"
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace ros;

unsigned short crc;
crcModbus crc_obj;

uint8_t read_buf;
string read;
uint8_t buf[8];
Serial driver("/dev/ttyUSB0", 115200);

class Driver
{
    private:
        ros::NodeHandle n;
        ros::ServiceServer ControlMod;
        ros::ServiceServer CurrentPosRet;
        ros::ServiceServer GoalPosReq;
        ros::Publisher CurrentVelRet;
        ros::Subscriber GoalVelReq;

    public:

        hexar_msgs::mod mod_obj;
        hexar_msgs::pos pos_obj;
        geometry_msgs::Twist cmd_vel;

        Driver()
        {
            ControlMod = n.advertiseService("ControlMod", &Driver::ControlModCallback, this);
            CurrentPosRet = n.advertiseService("CurrentPosRet", &Driver::CurrentPosRetCallback, this);
            GoalPosReq = n.advertiseService("GoalPosReq", &Driver::GoalPosReqCallback, this);
            CurrentVelRet = n.advertise<geometry_msgs::Twist>("CurrentVelRet", 10);
            GoalVelReq = n.subscribe("/cmd_vel", 1, &GoalVelReqCallback, this);
        }   

        bool ControlModCallback(hexar_msgs::mod::Request &req, hexar_msgs::mod::Response &res);
        bool CurrentPosRetCallback(hexar_msgs::pos::Request &req, hexar_msgs::pos::Response &res);
        bool GoalPosReqCallback(hexar_msgs::pos::Request &req, hexar_msgs::pos::Response &res);
        void GoalVelReqCallback(const geometry_msgs::Twist &vel_msg);
        
};

inline bool Driver::ControlModCallback(hexar_msgs::mod::Request &req, hexar_msgs::mod::Response &res)
{

    if(req.mod==1)//position control(relative)
    {
        res.mod_ena=true;
        
        ROS_WARN("controlMod-position(rel)");	    
        
        return true;
    }
    else if(req.mod==2)//position control(absolute)
    {
        res.mod_ena=true;
        ROS_WARN("controlMod-position(abs)");
        return true;
    }
    else if(req.mod==3)//speed control
    {
        res.mod_ena=true;
        buf[0] = 0x01;
        buf[1] = 0x06;
        buf[2] = 0x20;
        buf[3] = 0x32;
        buf[4] = 0x00;
        buf[5] = 0x03;
        buf[6] = 0x00;      //0x63
        buf[7] = 0x00;      //0xc4
        crc = crc_obj.crc_modbus(buf, 6);
        memcpy(&buf[6], (char*)&crc, 2);
        if(write(driver.serial_port, buf, 8)>0)
        ROS_WARN("controlMod-speed");
        return true;
    }
    else//invalid data
    {
        res.mod_ena=false;
        ROS_WARN("invalid mod num %d! please request valid data", req.mod);
        ROS_WARN("1: position(rel)");
        ROS_WARN("2: position(abs)");
        ROS_WARN("3: speed");
        return false;
    }

}

inline bool CurrentPosRetCallback(hexar_msgs::pos::Request &req, hexar_msgs::pos::Response &res)
{
    res.pos_ena=true;
    return true;
}

inline bool GoalPosReqCallback(hexar_msgs::pos::Request &req, hexar_msgs::pos::Response &res)
{
    res.pos_ena=true;
    return true;
}

inline void GoalVelReqCallback(const geometry_msgs::Twist &vel_msg)
{
    double goalVel = vel_msg.linear.x *10;
    goalVel

}



int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "zlac8015_driver_node");
    Driver Driver_obj;

    ros::spinOnce();
    return 0;
}


