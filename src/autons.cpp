#include "EZ-Template/util.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "subsystems.hpp"
#include "autons.hpp"
#include <cstdlib>
#include <string>

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DS = 100;
const int TS = 60;
const int SS = 110;
std::string eject_color = ""; // "red" or "blue"

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DS, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DS);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DS);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TS);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TS);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TS);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DS, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TS);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TS);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TS);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DS, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DS);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TS);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TS);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TS);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DS);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SS, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SS, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SS, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SS, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DS, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TS);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TS);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TS);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, TS, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DS, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TS);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SS, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TS);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DS, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DS, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TS);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DS, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DS);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DS);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DS},
                        {{0_in, 20_in}, fwd, DS},
                        {{0_in, 30_in}, fwd, DS}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DS},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DS},
                        {{12_in, 24_in}, fwd, DS},
                        {{24_in, 24_in}, fwd, DS}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DS},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DS},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DS},
                        {{12_in, 24_in}, fwd, DS},
                        {{24_in, 24_in}, fwd, DS}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DS},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .
void blue_negative_awp() {
  /*
  place robot in front of alliancestake
  score preload
  go to negative goal
  clamp goal
  get closest ring
  sweep corner
  intake those
  drive to touch hang
  */
  eject_color = "red";
  int64_t failsafe = pros::millis();
  lbPID.target_set(180);
  while (lb.get_position() < 160 && pros::millis() - failsafe < 500) {
    lb.move(lbPID.compute(lb.get_position()));
  }
  pros::delay(100);
  intake.move(127);
  pros::delay(500);
  chassis.pid_drive_set(8_in, DS);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, SS, ez::ccw);
  intakeLift.set(true);
  chassis.pid_wait();
  chassis.pid_drive_set(8_in, DS);
  chassis.pid_wait();
  intakeLift.set(false);
  failsafe = pros::millis();
  lbPID.target_set(975);
  while (lb.get_position() < 950 && pros::millis() - failsafe < 500) {
    lb.move(lbPID.compute(lb.get_position()));
    if (lb.get_position() <= 300) {
      hook.move(127);
    }
  }
  pros::delay(200);
  chassis.pid_drive_set(-8_in, DS);
  chassis.pid_wait();
  intake.move(0);
  failsafe = pros::millis();
  lbPID.target_set(0);
  while (lb.get_position() > 10 && pros::millis() - failsafe < 500) {
    lb.move(lbPID.compute(lb.get_position()));
  }
  chassis.pid_turn_set(-135_deg, TS);
  chassis.pid_wait();
  chassis.pid_drive_set(-30_in, DS);
  chassis.pid_wait();
  chassis.pid_drive_set(-6_in, DS/3);
  chassis.pid_wait();
  goalClamp.set(true);
  chassis.pid_drive_set(7_in, DS);
  chassis.pid_turn_set(90_deg, TS);
  chassis.pid_wait();
  pros::delay(200);
  intake.move(127);
  chassis.pid_drive_set(25_in, DS/1.5);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, SS/2, ez::ccw);
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, DS);
  chassis.pid_wait();
  chassis.pid_turn_set(-110_deg, TS);
  chassis.pid_wait(); 
  chassis.pid_drive_set(24_in, DS);
  chassis.pid_wait();
}

void blue_positive_awp() {
  /*
  place robot in front of alliancestake
  score preload
  go to mid goal
  extend goal doinker
  retract and pull back
  clamp goal
  get closest ring
  sweep corner
  intake those
  drive to touch hang
  */
  eject_color = "red";
}

void blue_goal_rush() {
  /*
  place robot in closest position to goal while touching starting tape
  extend doinker while driving max speed toward mid goal
  retract doinker when hits
  pull back
  enable tug detection
  if no tug, score preload
  if tug, realign
  release
  turn around and clamp other goal right next to it
  score top ring
  release near positive corner
  go for eyelash tech t1 contact
  */
  eject_color = "red";
  roller.move(-127);
  chassis.pid_drive_set(43_in, 127);
  chassis.pid_wait_until(30_in);
  goalrush.set(true);
  chassis.pid_wait_until(42_in);
  goalrush.set(false);
  chassis.pid_drive_set(-12_in, 127);
  chassis.pid_wait();
  goalrush.set(true);
  chassis.pid_drive_set(-5_in, DS);
  chassis.pid_wait();
  goalrush.set(false);
  chassis.pid_turn_set(210_deg, TS);
  chassis.pid_wait();
  chassis.pid_drive_set(-7_in, DS/2);
  chassis.pid_wait();
  goalClamp.set(true);
  intake.move(127);
  pros::delay(1500);
  intake.move(0);
  goalClamp.set(false);
  chassis.pid_drive_set(17_in, DS);
  chassis.pid_wait();
  chassis.pid_turn_set(135_deg, TS);
  chassis.pid_wait();
  chassis.pid_drive_set(-29_in, DS);
  chassis.pid_wait();
  chassis.pid_drive_set(-7_in, DS/2);
  chassis.pid_wait();
  goalClamp.set(true);
  intake.move(127);
  chassis.pid_drive_set(36_in, DS);
  chassis.pid_wait();
  chassis.pid_drive_set(18_in, DS/1.5);
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_drive_set(-5_in, DS);
  chassis.pid_wait();
  chassis.pid_turn_set(-45_deg, TS);
  chassis.pid_wait();
  chassis.pid_drive_set(38_in, DS);
}

void blue_top_rings() {

  /* ELIMS GOAL RUSH
  same starting code as goal rush, just add wallstakes at end instead of eyelash tech t1 contact
  */
  chassis.pid_wait();
  eject_color = "red";
}

//void blue_center_ring_rush() {
  /* ELIMS IF CANT GOAL RUSH GET STACKED ASAP ALSO NEED ALLIANCE WITH GOOD GOAL RUSH AND CAMPING SKILLS
  place robot in front of alliance stake
  score preload
  turn around and drive back to clamp goal
  turn and after turned, extend ring doinker
  get one red and drive back to intake and only move intake not hooks to hold it in robot
  turn and extend doinker again to get other ring
  drive back and get the negative side stacks on mid
  end at optimal pos for wall stakes
  */
//}

void red_negative_awp() {
  /*
  place robot in front of alliancestake
  score preload
  go to negative goal
  clamp goal
  get closest ring
  sweep corner
  intake those
  drive to touch hang
  */
  eject_color = "blue";
  int64_t failsafe = pros::millis();
  lbPID.target_set(180);
  while (lb.get_position() < 160 && pros::millis() - failsafe < 500) {
    lb.move(lbPID.compute(lb.get_position()));
  }
  pros::delay(100);
  intake.move(127);
  pros::delay(500);
  chassis.pid_drive_set(8_in, DS);
  chassis.pid_wait();
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, SS);
  intakeLift.set(true);
  chassis.pid_wait();
  chassis.pid_drive_set(8_in, DS);
  chassis.pid_wait();
  intakeLift.set(false);
  failsafe = pros::millis();
  lbPID.target_set(975);
  while (lb.get_position() < 950 && pros::millis() - failsafe < 500) {
    lb.move(lbPID.compute(lb.get_position()));
    if (lb.get_position() <= 300) {
      hook.move(127);
    }
  }
  pros::delay(200);
  chassis.pid_drive_set(-8_in, DS);
  chassis.pid_wait();
  intake.move(0);
  failsafe = pros::millis();
  lbPID.target_set(0);
  while (lb.get_position() > 10 && pros::millis() - failsafe < 500) {
    lb.move(lbPID.compute(lb.get_position()));
  }
  chassis.pid_turn_set(135_deg, TS);
  chassis.pid_wait();
  chassis.pid_drive_set(-30_in, DS);
  chassis.pid_wait();
  chassis.pid_drive_set(-6_in, DS/3);
  chassis.pid_wait();
  goalClamp.set(true);
  chassis.pid_drive_set(7_in, DS);
  chassis.pid_turn_set(90_deg, TS);
  chassis.pid_wait();
  pros::delay(200);
  intake.move(127);
  chassis.pid_drive_set(25_in, DS/1.5);
  chassis.pid_wait_quick();
  chassis.pid_swing_set(ez::LEFT_SWING, 90, SS/1.5);
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, DS);
  chassis.pid_wait();
  chassis.pid_turn_set(110_deg, TS);
  chassis.pid_wait(); 
  chassis.pid_drive_set(24_in, DS);
  chassis.pid_wait();
}

void red_positive_awp() {
  /*
  place robot in front of alliancestake
  score preload
  go to mid goal
  extend goal doinker
  retract and pull back
  clamp goal
  get closest ring
  sweep corner
  intake those
  drive to touch hang
  */
  eject_color = "red";
}

void red_goal_rush() {
  /*
  place robot in closest position to goal while touching starting tape
  extend doinker while driving max speed toward mid goal
  retract doinker when hits
  pull back
  enable tug detection
  if no tug, score preload
  if tug, realign
  release
  turn around and clamp other goal right next to it
  score top ring
  release near positive corner
  go for eyelash tech t1 contact
  */
  eject_color = "blue";
  chassis.pid_wait();
}

void red_top_rings() {
  /* ELIMS GOAL RUSH
  same starting code as goal rush, just add wallstakes at end instead of eyelash tech t1 contact
  */
  eject_color = "blue";
  chassis.pid_wait();
}

void skills() {
  //score the ring
  hook.move(127);
  pros::delay(700);
  hook.move(0);
  //swing to the right of 90 degrees
  chassis.pid_swing_set(ez::LEFT_SWING, 106.2_deg, SS);
  chassis.pid_wait();
  //Drive forward
  chassis.pid_drive_set(-32.56_in, 66);
  chassis.pid_wait();
  //Grab the goal
  goalClamp.set(true);
  chassis.pid_wait();
  //Turn to the left of -90 degrees
  chassis.pid_turn_set(0_deg, TS);
  chassis.pid_wait();
  //*****score the ring
  hook.move(128);
  intake.move(128);
  //Drive forward

  chassis.pid_drive_set(24,DS);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-12,DS);
  chassis.pid_wait();
  chassis.pid_turn_set(-22.7,TS);
  chassis.pid_wait();
  chassis.pid_drive_set(84,DS);
  chassis.pid_wait();

  //chassis.pid_swing_set(ez::RIGHT_SWING, -30, 75, 90);
  //chassis.pid_wait();
  //chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 90, 75);
  //chassis.pid_wait();
  //chassis.pid_turn_set(-30,TS);
  //chassis.pid_wait();
  //chassis.pid_drive_set(26.83,DS);
  //chassis.pid_wait();
  //Drive Back
  chassis.pid_drive_set(-26.83,DS);
  chassis.pid_wait();
  //Turn
  chassis.pid_turn_set(180_deg, TS);
  chassis.pid_wait();
  //Drive Forward
  chassis.pid_drive_set(84,DS);
  chassis.pid_wait();
  //Turn
  chassis.pid_turn_set(68,TS);
  chassis.pid_wait();
  intake.move(0);
  hook.brake();
  //Put the goal on the positive side
  chassis.pid_drive_set(-10,DS);
  chassis.pid_wait();
  goalClamp.set(false);
  chassis.pid_wait();
  chassis.pid_turn_set(5,TS);
  chassis.pid_wait();
  intake.move(127);
  chassis.pid_drive_set(16,DS);
  chassis.pid_wait();
  chassis.pid_turn_set(0,TS);
  chassis.pid_wait();

  chassis.pid_drive_set(48,DS);
  chassis.pid_wait();
  chassis.pid_turn_set(-90,TS);
  chassis.pid_wait();
  //score ring
  lbPID.target_set(180);
  lb.move(lbPID.compute(lb.get_position()));
  hook.move(127);
  pros::delay(500);
  lbPID.target_set(800);
  lb.move(lbPID.compute(lb.get_position()));
  chassis.pid_drive_set(-12,DS);
  lbPID.target_set(0);
  lb.move(lbPID.compute(lb.get_position()));
  chassis.pid_wait();

  chassis.pid_turn_set(56,TS);
  chassis.pid_wait();

  intake.move(127);
  hook.move(0);

  chassis.pid_turn_set(60,90);
  chassis.pid_wait();

  chassis.pid_drive_set(43.27,DS);
  chassis.pid_wait();

  chassis.pid_turn_set(180,TS);
  chassis.pid_wait();

  chassis.pid_drive_set(-37,66);
  chassis.pid_wait();

  goalClamp.set(true);
  chassis.pid_wait();

  chassis.pid_turn_set(100.8,TS);
  chassis.pid_wait();

  chassis.pid_drive_set(-48,DS);
  chassis.pid_wait();

  goalClamp.set(false);
  chassis.pid_wait();

  chassis.pid_turn_set(105.5,TS);
  chassis.pid_wait();

  chassis.pid_drive_set(51,DS);
  chassis.pid_wait();

  chassis.pid_turn_set(-60,TS);
  chassis.pid_wait();

  chassis.pid_drive_set(-5,DS);
  chassis.pid_wait();

  goalClamp.set(true);
  chassis.pid_wait();

  chassis.pid_turn_set(134.4,TS);
  chassis.pid_wait();

  chassis.pid_drive_set(-35,DS);
  chassis.pid_wait();

  goalClamp.set(true);
  chassis.pid_wait();
  hook.move(127);

  chassis.pid_turn_set(134.6,TS);
  chassis.pid_wait();
  chassis.pid_drive_set(35,DS);
  chassis.pid_wait();


  chassis.pid_turn_set(90,TS);
  chassis.pid_wait();

  chassis.pid_drive_set(25,DS);
  chassis.pid_wait();
  chassis.pid_turn_set(-180,TS);
  chassis.pid_wait();

  chassis.pid_drive_set(84,DS);
  chassis.pid_wait();

  intake.move(0);
  hook.brake();

  chassis.pid_turn_set(111,TS);
  chassis.pid_wait();
  chassis.pid_drive_set(-17,DS);
  chassis.pid_wait();
  goalClamp.set(false);
  chassis.pid_wait();

  chassis.pid_turn_set(-42,TS);
  chassis.pid_wait();

  chassis.pid_drive_set(55,DS);
  chassis.pid_wait();

  climbArm.set(true);
  chassis.pid_wait();

  chassis.pid_drive_set(10,30);
  chassis.pid_wait();

  lb.move(127);
}