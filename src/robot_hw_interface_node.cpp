#include "diff_drive_controller/robot_hw_interface_node.h"

MyRobotHWInterface::MyRobotHWInterface(ros::NodeHandle &nh) : nh_(nh)
{
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 10;

    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

    async_time = nh_.createTimer(update_freq, &MyRobotHWInterface::update, this);


}
MyRobotHWInterface::~MyRobotHWInterface(){}

void MyRobotHWInterface::init()
{
    for ( int i = 0; i < 1; ++i)
    {
        // create a joint interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // create a velocity joint interface
        hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

        // create a joint limit interface
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
        joint_limits_interface::VelocityJointSaturationHandle jointlimitsHandle(jointVelocityHandle, limits);
        velocity_joint_saturation_interface_.registerHandle(jointlimitsHandle);

    }

    // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocity_joint_saturation_interface_);

}
void MyRobotHWInterface::update(const ros::TimerEvent& te)
{

    ROS_INFO("update");

    elapsed_time_ = ros::Duration(te.current_real - te.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);

}
void MyRobotHWInterface::read()
{
    // uint8_t buff[4];

    // left_motor.readBytes(buff, 4);

    // int sign = (buff[0] & 0b10000000) == 0b10000000 ? -1 : 1;
    // double joint_velocity = (double)(((((buff[0] & 0b01111111) << 8) | (buff[1]))/100.0))*sign;

    // //double joint_velocity = (double)((buff[0] >> 8) | buff[1]);

    // joint_velocity_ = angles::from_degrees(joint_velocity);

    // double joint_position = (double)((buff[2] >> 8) | buff[3]);
    // joint_position_ = angles::from_degrees((double)((buff[2] >> 8) | buff[3]));
}
void MyRobotHWInterface::write(ros::Duration elapsed_time)
{

    velocity_joint_saturation_interface_.enforceLimits(elapsed_time);

}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "single_joint_interface");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    MyRobotHWInterface my_bot(nh);
    //spinner.start();
    spinner.spin();

    return 0;
}