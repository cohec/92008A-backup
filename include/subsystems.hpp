#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/colors.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include "pros/vision.hpp"

extern Drive chassis;
inline pros::MotorGroup intake({4,-5});
inline pros::Motor hook(-5);
inline pros::Motor lb(17);
inline pros::Optical op(2);
inline ez::Piston goalClamp('A');
inline ez::Piston climbArm('B');
inline ez::Piston goalrush('C');
inline ez::Piston ringrush('D');
inline ez::Piston intakeLift('E');
inline pros::Optical col(6);