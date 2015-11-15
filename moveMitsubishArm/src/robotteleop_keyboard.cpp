/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>

#include <moveMitsubishArm/KeyboardInput.h>


#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class RobotTeleop
{
public:
    RobotTeleop();
    void keyLoop();
    void watchdog();

private:


    ros::NodeHandle nh_,ph_, kInp_;
    float incX, incY, incZ, incRX, incRY, incRZ;
    ros::Time first_publish_;
    ros::Time last_publish_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    ros::Publisher keybInp_;
    ros::Subscriber pose_sub_ ;
    void publishInc(float, float, float, float, float, float);
    boost::mutex publish_mutex_;

};

RobotTeleop::RobotTeleop():
    ph_("~"),
    l_scale_(1.0),
    a_scale_(1.0)
{
    ph_.param("scale_angular", a_scale_, a_scale_);
    ph_.param("scale_linear", l_scale_, l_scale_);
    keybInp_ = kInp_.advertise<moveMitsubishArm::KeyboardInput>("position", 1000);
    //pose_sub_ = nh_.subscribe("/mavros/vision_pose/pose" ,1, &RobotTeleop::poseCallback, this );

}


int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_teleop");
    RobotTeleop keyboard_teleop;
    ros::NodeHandle n;

    //std::cout<<"qt funziona";

    signal(SIGINT,quit);

    boost::thread my_thread(boost::bind(&RobotTeleop::keyLoop, &keyboard_teleop));
    ros::Timer timer = n.createTimer(ros::Duration(0.01), boost::bind(&RobotTeleop::watchdog, &keyboard_teleop));
    ros::spin();
    my_thread.interrupt() ;
    my_thread.join() ;
    return(0);
}


void RobotTeleop::watchdog()
{
    boost::mutex::scoped_lock lock(publish_mutex_);
    if ((ros::Time::now() > last_publish_ + ros::Duration(0.01)) &&
            (ros::Time::now() > first_publish_ + ros::Duration(0.01)))
        publishInc(0, 0, 0, 0, 0, 0);
}

void RobotTeleop::keyLoop()
{
    char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("  ");
    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robotic arm.");


    while (ros::ok())
    {
      
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        incX =incY =incZ =incRX =incRY =incRZ =0;
        
        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c)
        {
	
        case KEYCODE_L:
            ROS_INFO("LEFT");
            incX = 0.0;
            incY = 0.01;
            incZ = 0.0;
            incRX = 0.0;
            incRY = 0.0;
            incRZ = 0.0;


            break;
        case KEYCODE_R:
            ROS_INFO("RIGHT");
            incX = 0.0;
            incY = -0.01;
            incZ = 0.0;
            incRX = 0.0;
            incRY = 0.0;
            incRZ = 0.0;

            break;
        case KEYCODE_U:
            ROS_DEBUG("UP");
            incX = 0.01;
            incY = 0.0;
            incZ = 0.0;
            incRX = 0.0;
            incRY = 0.0;
            incRZ = 0.0;

            break;
        case KEYCODE_D:
            ROS_DEBUG("DOWN");
            incX = -0.01;
            incY = 0.0;
            incZ = 0.0;
            incRX = 0.0;
            incRY = 0.0;
            incRZ = 0.0;

            break;

        case '8':
            ROS_DEBUG("rz+");
            incX = 0.0;
            incY = 0.0;
            incZ = 0.0;
            incRX = 0.0;
            incRY = 0.0;
            incRZ = 0.1;

            break;

        case '2':
            ROS_DEBUG("rz-");
            incX = 0.0;
            incY = 0.0;
            incZ = 0.0;
            incRX = 0.0;
            incRY = 0.0;
            incRZ = -0.1;

            break;//*/

        case '4':
            ROS_DEBUG("ry+");
            incX = 0.0;
            incY = 0.0;
            incZ = 0.0;
            incRX = 0.0;
            incRY = 0.1;
            incRZ = 0.0;

            break;

        case '6':
            ROS_DEBUG("ry-");
            incX = 0.0;
            incY = 0.0;
            incZ = 0.0;
            incRX = 0.0;
            incRY = -0.1;
            incRZ = 0.0;

            break;//*/


        case KEYCODE_Q:
            ROS_DEBUG("Emergency");
            break;
        }
        boost::mutex::scoped_lock lock(publish_mutex_);
        if (ros::Time::now() > last_publish_ + ros::Duration(0.01)) {
            first_publish_ = ros::Time::now();
        }
        last_publish_ = ros::Time::now();
        publishInc(incX, incY, incZ, incRX, incRY, incRZ);
    }

    return;
}

void RobotTeleop::publishInc(float incX,float incY,float incZ, float incRX, float incRY,float incRZ)  
{
    
    moveMitsubishArm::KeyboardInput msg;
    msg.header.stamp = ros::Time::now();

    msg.x = incX;
    msg.y = incY;
    msg.z = incZ;
    msg.rx = incRX;
    msg.ry = incRY;
    msg.rz = incRZ;

    keybInp_.publish(msg);
   
    return;
}




