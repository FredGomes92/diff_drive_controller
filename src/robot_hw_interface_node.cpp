#include "diff_drive_controller/robot_hw_interface_node.h"

MyRobotHWInterface::MyRobotHWInterface(ros::NodeHandle &nh) : nh_(nh)
{
    init();


    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 10;

    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);

    async_time = nh_.createTimer(update_freq, &MyRobotHWInterface::update, this);

    pub_enc_pos_left = nh_.advertise<std_msgs::Int16>("encoder_pos_left", 1000);
    pub_enc_pos_right = nh_.advertise<std_msgs::Int16>("encoder_pos_right", 1000);
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
    int left, right;

    left_motor.readBytes(rbuff,1);
    left=(int8_t)rbuff[0];
    left_motor_pos += angles::from_degrees((double)left);
    joint_position_[0]=left_motor_pos;

    right_motor.readBytes(rbuff,1);
    right=(int8_t)rbuff[0];
    right_motor_pos += angles::from_degrees((double)right);
    joint_position_[1]=right_motor_pos;

    

    //ROS_INFO("joint_psition[0]: %f, joint_psition[01]: %f", joint_position_[0], joint_position_[1]);
    // ROS_INFO("lll: %f, right: %f", left, right);
    

    ComputePosition(left,right);


}
void MyRobotHWInterface::write(ros::Duration elapsed_time)
{

    velocity_joint_saturation_interface_.enforceLimits(elapsed_time);

    uint8_t wbuff[2];

    int velocity_left, velocity_right,result;

    velocity_left=(int)angles::to_degrees(joint_velocity_command_[0]);
	wbuff[0]=velocity_left;
    wbuff[1]=velocity_left >> 8;

    ROS_INFO("joint_velocity_command_[0]=%.2f velocity_left=%d  wbuff[0]=%d wbuff[1]=%d", joint_velocity_command_[0],velocity_left,wbuff[0],wbuff[1]);

    if(right_prev_cmd!=velocity_left)
    {
	    result = right_motor.writeData(wbuff,2);

        ROS_INFO("Writen successfully result=%d", result);
	    right_prev_cmd = velocity_left;
    }

    velocity_right=(int)angles::to_degrees(joint_velocity_command_[1]);
	wbuff[0]=velocity_right;
    wbuff[1]=velocity_right >> 8;

    ROS_INFO("joint_velocity_command_[1]=%.2f velocity_right=%d  wbuff[0]=%d wbuff[1]=%d", joint_velocity_command_[1],velocity_right,wbuff[0],wbuff[1]);

    if(lef_prev_cmd!=velocity_right)
    {
	    result = left_motor.writeData(wbuff,2);

        ROS_INFO("Writen successfully result=%d", result);
	    lef_prev_cmd = velocity_right;
    }


}

void MyRobotHWInterface::ComputePosition(int pos_left_inc, int pos_right_inc)
{

    left_pos =  left_pos + pos_left_inc;
    right_pos =  right_pos + pos_right_inc;

    // When the motor goes from fowrard to backwards, we need to guarantee that the counter will be always negative
    // Eg: Stopped at 280º --> goes to 270º ---> should display position -10º!
    if ((pos_left_inc < 0) && (left_pos > 0))
    {
        left_pos = pos_left_inc;
    }
    if ((pos_right_inc < 0) && (right_pos > 0))
    {
        right_pos = pos_right_inc;
    }

    // When the motor goes from fowrard to backwards, we need to guarantee that the counter will be always negative
    // Eg: Stopped at -280º --> goes to -270º ---> should display position 10º!
    if ((pos_left_inc > 0) && (left_pos < 0))
    {
        left_pos = pos_left_inc;
    }
    if ((pos_right_inc > 0) && (right_pos < 0))
    {
        right_pos = pos_right_inc;
    }

    //------------------------------

    if (left_pos < 0)
    {
        left_pos = abs(left_pos%360)*-1;
    }
    else{
        left_pos = left_pos%360;
    }

    if (right_pos < 0)
    {
        right_pos = abs(right_pos%360)*-1;
    }
    else{
        right_pos = right_pos%360;
    }
    
    enc_pos_left.data = left_pos;
    enc_pos_right.data = right_pos;

    pub_enc_pos_left.publish(enc_pos_left);
    pub_enc_pos_right.publish(enc_pos_right);

    // ROS_INFO("pos left_inc: %d, pos right_inc: %d", pos_left_inc, pos_right_inc);

    // ROS_INFO("pos left: %d, pos right: %d", left_pos, right_pos);

    // ros::Duration(1.0).sleep();


}

int main(int argc, char ** argv)
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "single_joint_interface");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2); // Multiple threads for controller service callback and for the Service client callback used to get the feedback from ardiuno
    MyRobotHWInterface my_bot(nh);
    //spinner.start();
    spinner.spin();

    return 0;
}