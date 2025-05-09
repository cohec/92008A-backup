#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"
#include <string>

extern Drive chassis;
extern std::string eject_color;
inline pros::MotorGroup intake({4,-5});
inline pros::Motor hook(-5);
inline pros::Motor lb(17);
inline pros::Optical op(6);
inline ez::Piston goalClamp('C');
inline ez::Piston climbArm('D');
inline ez::Piston goalrush('E');
inline ez::Piston ringrush('F');
inline ez::Piston intakeLift('G');
inline ez::PID lbPID{0.45, 0, 0, 0, "lb"};
void sortcolor(bool enabled);
void antijam(int direction);