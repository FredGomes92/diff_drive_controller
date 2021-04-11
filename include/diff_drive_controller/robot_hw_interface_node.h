#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>

#include "diff_drive_controller/i2c.h"

class MyRobotHWInterface : public hardware_interface::RobotHW
{
public:
    MyRobotHWInterface(ros::NodeHandle& nh);
    ~MyRobotHWInterface();

    void init();
    void update(const ros::TimerEvent& te);
    void read();
    void write(ros::Duration elapsed_time);

protected:

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    joint_limits_interface::VelocityJointSaturationInterface velocity_joint_saturation_interface_;

    std::string joint_name_[2] = {"left_wheel_joint", "right_wheel_joint"};

    double joint_position_[2];
    double joint_velocity_[2];
    double joint_effort_[2];
    double joint_velocity_command_[2];

    double left_motor_pos = 0.0, right_motor_pos = 0.0;

    i2c::I2C right_motor = i2c::I2C(1,10);
    i2c::I2C left_motor = i2c::I2C(1,11);

    ros::NodeHandle nh_;
    ros::Timer async_time;
    ros::Duration elapsed_time_;

    double loop_hz_;

    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

private:
    int lef_prev_cmd = 0, right_prev_cmd = 0;

};