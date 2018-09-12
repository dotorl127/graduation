// turtlebot3_teleop에서 /cmd_vel
// plactice1_pkg에서 /state
// 두 토픽을 서브스크라이빙해서 스테이트를 정하고
// turtlebot을 제어하는 /cmd_vel_proc을 퍼블리싱
// mobing object 추가(image_processor에 넣고 sensor_processor라고 노드이름 바꿔도 괜찮을 듯)

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <string>
#include <unistd.h>

using namespace std;

class state_machine
{
    private:
    geometry_msgs::Twist pubCmdMsg;
    string state;
    bool stopflag = true;
    double begin = 0, end = 0;

    public:
    ros::NodeHandle nh_;
    ros::Subscriber sub_state;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_sensor;
    ros::Publisher pub_vel;

    state_machine();
    ~state_machine();
    void cmdCallback(const geometry_msgs::Twist& cmd_vel_msg);
    void stateCallback(const std_msgs::String& state_msg);
    void sensorCallback(const sensor_msgs::LaserScan& scan_msg);
    void stateProcess();
};

state_machine::state_machine()
{
    pubCmdMsg.linear.x = pubCmdMsg.linear.y = pubCmdMsg.linear.z = 0;
    pubCmdMsg.angular.x = pubCmdMsg.angular.y = pubCmdMsg.angular.z = 0;
    state = "";

    sub_state = nh_.subscribe("state", 1, &state_machine::stateCallback, this);
    sub_vel = nh_.subscribe("cmd_vel", 1, &state_machine::cmdCallback, this);
    sub_sensor = nh_.subscribe("scan", 1, &state_machine::sensorCallback, this);
    pub_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel_proc", 1);
}
state_machine::~state_machine() {}

void state_machine::cmdCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
    pubCmdMsg = cmd_vel_msg;
    stateProcess();
}

void state_machine::stateCallback(const std_msgs::String& state_msg)
{
    if(state_msg.data != "")
        state = state_msg.data;
}

void state_machine::sensorCallback(const sensor_msgs::LaserScan& scan_msg)
{
    for(int i = 10; i <= 0; i--)
    {
        if(scan_msg.ranges[i] <= 1.0) state = "movingObj";
        else state = "";
    }
    for(int i = 350; i <= 360; i++)
    {
        if(scan_msg.ranges[i] <= 1.0) state = "movingObj";
        else state = "";
    }
}

void state_machine::stateProcess()
{
    if(state == "speed10")
    {
        if(pubCmdMsg.linear.x >= 0.1) pubCmdMsg.linear.x = 0.1;
        stopflag = true;
    }
    else if(state == "speed20")
    {
        if(pubCmdMsg.linear.x >= 0.2) pubCmdMsg.linear.x = 0.2;
        stopflag = true;
    }
    else if(state == "stop" && stopflag == true)
    {
        pubCmdMsg.linear.x = 0;
        pubCmdMsg.linear.y = 0;
        pubCmdMsg.linear.z = 0;
        pubCmdMsg.angular.z = 0;

        pub_vel.publish(pubCmdMsg);
        
        ros::Duration(2).sleep();

        stopflag = false;
    }
    else if(state == "movingObj" || state == "trafficLightRed")
    {
        pubCmdMsg.linear.x = 0;
        pubCmdMsg.linear.y = 0;
        pubCmdMsg.linear.z = 0;
        pubCmdMsg.angular.z = 0;

        pub_vel.publish(pubCmdMsg);
    }
    else ;

    pub_vel.publish(pubCmdMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_machine");

    state_machine one;

    ros::spin();

    return 0;
}