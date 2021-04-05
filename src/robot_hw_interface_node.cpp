#include "ros_drive_controller/robot_hw_interface_node.h"

MyRobotHWInterface::MyRobotHWInterface(ros::NodeHandle &nh) : nh_(nh)
{
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 5;

    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    async_time = nh_.createTimer(update_freq, &MyRobotHWInterface::update, this);

    motor_pos = 0;

}
MyRobotHWInterface::~MyRobotHWInterface(){}

void MyRobotHWInterface::init()
{
    joint_name_ = "joint1";
    // create a joint interface
    hardware_interface::JointStateHandle jointStateHandle(joint_name_, &joint_position_, &joint_velocity_, &joint_effort_);
    joint_state_interface_.registerHandle(jointStateHandle);

    // create a velocity joint interface
    // hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_);
    // velocity_joint_interface_.registerHandle(jointVelocityHandle);

    // create an effort joint interface
    hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_);
    effort_joint_interface_.registerHandle(jointEffortHandle);

    // create a joint limit interface
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits("joint1", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointlimitsHandle(jointEffortHandle, limits);
    effortJointSaturationInterface_.registerHandle(jointlimitsHandle);

    // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&effortJointSaturationInterface_);

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
    uint8_t buff[4];

    my_motor.readBytes(buff, 4);

    int sign = (buff[0] & 0b10000000) == 0b10000000 ? -1 : 1;
    double joint_velocity = (double)(((((buff[0] & 0b01111111) << 8) | (buff[1]))/100.0))*sign;

    //double joint_velocity = (double)((buff[0] >> 8) | buff[1]);

    joint_velocity_ = angles::from_degrees(joint_velocity);

    double joint_position = (double)((buff[2] >> 8) | buff[3]);
    joint_position_ = angles::from_degrees((double)((buff[2] >> 8) | buff[3]));

    ROS_INFO("Vel (rad): %f, (deg): %f,  Pos (rad): %f, Pos (deg): %f", joint_velocity_, joint_velocity,  joint_position_, joint_position);
    ROS_INFO("Buff[0]: %d, Buff[1]: %d", buff[0], buff[1]);

    ROS_INFO ("Joint effort command: %f", joint_effort_command_);


}
void MyRobotHWInterface::write(ros::Duration elapsed_time)
{

    effortJointSaturationInterface_.enforceLimits(elapsed_time);
    // double vel = 150;
    // uint8_t buff[4];

    // buff[0]=velocity;
    // buff[1]=velocity >> 8;

    // buff[2] = 0;
    // buff[3] = 0;

    uint8_t buff[4];

    int tmp = (int)(joint_effort_command_*100);

    buff[0] = abs(tmp) >> 8; // MSB
    buff[1] = abs(tmp); // LSB
    buff[2] = 0;
    buff[3] = 0;

    if (tmp < 0) // 1st bit is 1 if the command is negative, otherwise, it will be 0
        buff[0] = buff[0] | 0b10000000;
    else
        buff[0] = buff[0] | 0b00000000;

    if (motor_prev_cmd != joint_effort_command_)
    {
        motor_prev_cmd = joint_effort_command_;
        my_motor.writeData(buff, 4);
    }

    ROS_INFO("PWM Cmd: %.2f",joint_effort_command_);
    //ROS_INFO("tmp: %d , Buff[0]: %d, buff[1]: %d", tmp, buff[0], buff[1]);
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