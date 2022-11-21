#ifndef GOTOPOSESACTIONSERVER_H
#define GOTOPOSESACTIONSERVER_H

#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <diff_drive_controller/GoToPosesActionServer.h>
#include <ros/console.h>

#include <my_diff_drive_controller/GoTo2DPoseAction.h>
#include <my_diff_drive_controller/GoTo2DPoseGoal.h>
#include <my_diff_drive_controller/GoTo2DPoseResult.h>
#include <my_diff_drive_controller/GoTo2DPoseFeedback.h>
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <vector>

enum class move_type
{
    FORWARD, 
    BACKWARDS,
    STOP, 
    ROTATE_LEFT,
    ROTATE_RIGHT
};


struct Move
{
    
    static unsigned _pos_left;
    static unsigned _pos_right;


    Move(const unsigned &dist, const move_type &m, ros::NodeHandle _nh): _dist(dist), _move(m), nh_(_nh)
    {
        pub_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
        _last_pos_left = 0;
        _ticks_enc_left = 0;

        Move::_pos_left = 0;        
        
    }

    void Start()
    {
        switch (_move)
        {
            case move_type::FORWARD:
            msg.linear.x = 0.5;
            msg.angular.z = 0.0;
            break;
            case move_type::BACKWARDS:
            msg.linear.x = -0.5;
            msg.angular.z = 0.0;
            break;
            case move_type::STOP:
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            break;
            case move_type::ROTATE_LEFT:
            msg.linear.x = 0.0;
            msg.angular.z = -1;
            break;
            case move_type::ROTATE_RIGHT:
            msg.linear.x = 0.0;
            msg.angular.z = 1;
            break;
            default:
            break;
        }
        pub_vel.publish(msg);

        while(not HasFinished()); // block until movement is finished

        msg.linear.x = 0.0;
        msg.angular.z = 0.0;

        pub_vel.publish(msg);
    }

    static void Subscribe_pos_left_wheel_cb(const std_msgs::Int16::ConstPtr &pos)
    {
        // std::stringstream ss;
        // ss << "pos_left: " << pos->data << "\n";
        // ROS_WARN_STREAM("pos_left: " << pos->data);

        Move::_pos_left = abs(pos->data);

    }
    static void Subscribe_pos_right_wheel_cb(const std_msgs::Int16::ConstPtr &pos)
    {
        // ss << "pos_left: " << pos->data << "\n";
        // ROS_WARN_STREAM("pos_right: " << pos->data);

        Move::_pos_right = abs(pos->data);

    }
    ros::NodeHandle nh_;

    ~Move(){}

    private:

        int _dist;

        ros::Publisher pub_vel;

        geometry_msgs::Twist msg;
        move_type _move;

        unsigned _last_pos_left;
        unsigned _ticks_enc_left;

        bool _first_update = true;

        bool HasFinished()
        {
            
            pub_vel.publish(msg);
            
            std::string p;


            // Discard the first measure, since in case the robot is changing direction it will fail: 
            //          Eg: Moving forward: stopped at pos_left = 280º
            //          Eg: Start moving Backwards: pos_left = 10 --> should count 10 and not (360-280) + 10

            if ((Move::_pos_left != _last_pos_left) && (_first_update))
            {
                _first_update = false;
                _last_pos_left = Move::_pos_left;
                return false;    
            }
            
            
            else if ((Move::_pos_left != _last_pos_left) && (!_first_update))
            {
                if (Move::_pos_left < _last_pos_left) // has started a new turn
                {
                    p = " 1";
                    _ticks_enc_left = _ticks_enc_left + (360 - _last_pos_left) + Move::_pos_left;
                    _last_pos_left = Move::_pos_left; 
                }
                else
                {
                    p = " 2";
                    _ticks_enc_left = _ticks_enc_left + (Move::_pos_left - _last_pos_left);
                    _last_pos_left = Move::_pos_left; 
                }
            }

            ROS_WARN_STREAM("ticks_left: " << _ticks_enc_left << " last_ticks_left: " << _last_pos_left << " linear_dist_ticks: " << _dist << "--- " << Move::_pos_left << p);


            // ros::Duration(0.2).sleep();
            return ((_ticks_enc_left >= _dist) || (_move == move_type::STOP));          
        }

        template <typename Enumeration>
        auto as_integer(Enumeration const value)
            -> typename std::underlying_type<Enumeration>::type
        {
            return static_cast<typename std::underlying_type<Enumeration>::type>(value);
        }
};

unsigned Move::_pos_left;
unsigned Move::_pos_right;


class GoToPosesActionServer
{
public:
    GoToPosesActionServer(const std::string &name);

    const std::string& GetActionName();
    
    ~GoToPosesActionServer();

private:

    ros::Subscriber sub_enc_pos_left;
    ros::Subscriber sub_enc_pos_right;
    ros::Publisher pub_vel;

    void ExecuteCallBack(const my_diff_drive_controller::GoTo2DPoseGoal::ConstPtr &goal);
    void Subscribe_pos_left_wheel_cb(const std_msgs::Int16::ConstPtr &pos);
    void Subscribe_pos_right_wheel_cb(const std_msgs::Int16::ConstPtr &pos);

    bool PointReached(int &ticks);
    void Pub_vel(move_type m);

    int pos_left; 
    int pos_right;

    std::stringstream ss;


protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<my_diff_drive_controller::GoTo2DPoseAction> as_;
    std::string _action_name;

    my_diff_drive_controller::GoTo2DPoseFeedback feedback_;
    my_diff_drive_controller::GoTo2DPoseResult result_;
    
};

#endif