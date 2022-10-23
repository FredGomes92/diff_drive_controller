#include "read_wheels_position.h"

read_wheels_position::read_wheels_position()
{

}

read_wheels_position::~read_wheels_position()
{

}

read_wheels_position::Read()
{
    uint8_t rbuff[1];
    int x;

    right_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    right_motor_pos += angles::from_degrees((double)x);

    left_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    left_motor_pos += angles::from_degrees((double)x);
    

    ROS_INFO("joint_position[0] (right): %f, joint_position[1] (left): %f", right_motor_pos, left_motor_pos);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_wheels_position");

    ros::NodeHandle nh;

    // create a topic to publish the encoders' position
    std_msgs::Int8MultiArray enc_pos;

    while(ros::ok())
    {

        enc_pos.data[0] = right_motor_pos;
        enc_pos.data[1] = left_motor_pos;

        ros::Publisher enc_pos_publisher = nh.advertise<std_msgs::Int8MultiArray>("encoder_pos", 1000);
        
        enc_pos_publisher.publish(enc_pos);

        spinner.spin();
        ros::spinOnce();
    
    }

    return 0;
}