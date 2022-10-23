#ifndef read_wheels_position_H
#define read_wheels_position_H

#pragma once

#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>
#include "diff_drive_controller/i2c.h"


class read_wheels_position
{
public:
    read_wheels_position();
    ~read_wheels_position();

private:
    i2c::I2C right_motor = i2c::I2C(1,10);
    i2c::I2C left_motor = i2c::I2C(1,11);
    void Read();
    float left_motor_pos, right_motor_pos;


};

#endif