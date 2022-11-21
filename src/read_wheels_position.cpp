#include "diff_drive_controller/read_wheels_position.h"

read_wheels_position::read_wheels_position()
{

}

read_wheels_position::~read_wheels_position()
{

}

void read_wheels_position::Read()
{
    uint8_t rbuff[1];
    int x;

    right_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    right_motor_pos += (double)x;

    left_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    left_motor_pos += (double)x;
    

    ROS_INFO("joint_position[0] (right): %f, joint_position[1] (left): %f", right_motor_pos, left_motor_pos);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_wheels_position");

    ros::NodeHandle nh;

    // create a topic to publish the encoders' position
    std_msgs::Int8MultiArray enc_pos;

    read_wheels_position read_wheels_pos;

    read_wheels_pos.enc_pos_publisher = nh.advertise<std_msgs::Int8MultiArray>("encoder_pos", 1000);

    while(ros::ok())
    {

        // enc_pos.data[0] = 10;
        // enc_pos.data[1] = 5;

        // ROS_INFO("1");

        read_wheels_pos.Read();

        // ROS_INFO("2");

        

        
        read_wheels_pos.enc_pos_publisher.publish(enc_pos);

        ros::spinOnce();
    
    }

    return 0;
}