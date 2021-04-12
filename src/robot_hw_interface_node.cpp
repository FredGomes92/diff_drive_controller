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
    for ( int i = 0; i < 2; ++i)
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
    uint8_t rbuff[1];
    int x;

    right_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    right_motor_pos += angles::from_degrees((double)x);
    joint_position_[0]=right_motor_pos;

    ROS_INFO("x=%d ",x);
}
void MyRobotHWInterface::write(ros::Duration elapsed_time)
{

    velocity_joint_saturation_interface_.enforceLimits(elapsed_time);

    uint8_t wbuff[2];

    int velocity,result;

    velocity=(int)angles::to_degrees(joint_velocity_command_[0]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;

    ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  wbuff[0]=%d wbuff[1]=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(right_prev_cmd!=velocity)
    {
	    result = right_motor.writeData(wbuff,2);

        ROS_INFO("Writen successfully result=%d", result);
	    right_prev_cmd = velocity;
    }

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