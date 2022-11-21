#include "diff_drive_controller/GoToPosesActionServer.h"

GoToPosesActionServer::GoToPosesActionServer(const std::string &name):
    _action_name(name),
    as_ (nh_, name, boost::bind(&GoToPosesActionServer::ExecuteCallBack, this, _1), false)
{
    sub_enc_pos_left = nh_.subscribe("/encoder_pos_left", 1000, &Move::Subscribe_pos_left_wheel_cb);
    sub_enc_pos_right = nh_.subscribe("/encoder_pos_right", 1000, &Move::Subscribe_pos_right_wheel_cb);
    
    pub_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    pos_left = 0;
    pos_right = 0;

    ROS_INFO("Action server has started!");

    as_.start();

}

void GoToPosesActionServer::ExecuteCallBack(const my_diff_drive_controller::GoTo2DPoseGoal::ConstPtr &goal)
{
    ROS_INFO("Goal received!");

    bool succeed = false;

    std::vector<Move*> vect;

    vect.push_back(new  Move(100, move_type::FORWARD, nh_));
    vect.push_back(new  Move(90, move_type::ROTATE_RIGHT, nh_));
    vect.push_back(new  Move(100, move_type::FORWARD, nh_));
    vect.push_back(new  Move(90, move_type::ROTATE_RIGHT, nh_));
    vect.push_back(new  Move(100, move_type::FORWARD, nh_));
    vect.push_back(new  Move(90, move_type::ROTATE_RIGHT, nh_));
    vect.push_back(new  Move(100, move_type::FORWARD, nh_));


    // vect.push_back(new  Move(360, move_type::ROTATE_RIGHT, nh_));

    // vect.push_back(new  Move(360, move_type::STOP, nh_));
    // vect.push_back(new  Move(360, move_type::ROTATE_RIGHT, nh_));

    // vect.push_back(new  Move(360, move_type::STOP, nh_));




    
    for (size_t i = 0; i < goal->points.size(); ++i)
    {

        if(as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", _action_name.c_str());
            // set the action state to preempted
            succeed = false;
            break;
        }

        for (std::vector<Move*>::iterator it = vect.begin(); it != vect.end(); ++ it)
        {
            (*it)->Start();
            ros::Duration(1.0).sleep();
        }
  

        // for (auto move=vect.begin(); i < vect.end(); i++)
        // {
            // &move->Start();
        // }



        // Move *m_1 = new  Move(1000, move_type::BACKWARDS, nh_);
        // Move *m_2 = new  Move(1000, move_type::FORWARD, nh_);
        // Move *m_3 = new  Move(360, move_type::ROTATE_RIGHT, nh_);
        // Move *m_4 = new  Move(360, move_type::ROTATE_LEFT, nh_);
        // Move *m_5 = new Move(720, move_type::ROTATE_RIGHT, nh_);
        // Move *m_6 = new Move(720, move_type::ROTATE_LEFT, nh_);
        // Move *m_7 = new Move(720, move_type::ROTATE_RIGHT, nh_);
        // Move *m_8 = new Move(720, move_type::ROTATE_LEFT, nh_);
        // Move *m_9 = new Move(720, move_type::ROTATE_RIGHT, nh_);
        // Move *m_10 = new Move(720, move_type::ROTATE_LEFT, nh_);

        
        // // m_1->Start();
        // // m_2->Start();
        // // m_3->Start();
    
        // // m_4->Start();
        // m_5->Start();
        // m_6->Start();
        // m_7->Start();
        // m_8->Start();
        // m_9->Start();
        // m_10->Start();
        

        feedback_.succeed = succeed;
        feedback_.feedback = ss.str();

        as_.publishFeedback(feedback_);
    }

    //feedback_.succeed = true;
    feedback_.feedback = "Robot has moved " + std::to_string(pos_left);

    as_.publishFeedback(feedback_); 

    ros::Duration(2.0).sleep();  // Sleep for one second

    result_.result = "Success!!!";
    as_.setSucceeded(result_);

}
   


void GoToPosesActionServer::Subscribe_pos_left_wheel_cb(const std_msgs::Int16::ConstPtr &pos)
{
    pos_left = pos->data;
    std::cout << "##################################\n";
}

void GoToPosesActionServer::Subscribe_pos_right_wheel_cb(const std_msgs::Int16::ConstPtr &pos)
{        
}

const std::string& GoToPosesActionServer::GetActionName()
{
    return _action_name;
}

bool GoToPosesActionServer::PointReached(int &ticks)
{

    // std::cout << "ticks" << ticks << std::endl;
    return (ticks >= 720);
}

void GoToPosesActionServer::Pub_vel(move_type m)
{
    geometry_msgs::Twist msg;
    
    switch (m)
    {
        case move_type::FORWARD:
        msg.linear.x = 0.5;
        msg.angular.z = 0.0;
        break;
        case move_type::BACKWARDS:
        msg.angular.x = -0.5;
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
}

GoToPosesActionServer::~GoToPosesActionServer()
{
    Pub_vel(move_type::STOP);
}

int main(int argc, char ** argv)
{
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "go_to_poses_action_srv");

    GoToPosesActionServer go_to_poses ("go_to_poses");

    ros::Rate rate(1);


    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}