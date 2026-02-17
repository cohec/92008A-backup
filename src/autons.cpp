#include "EZ-Template/util.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.h"
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
//std::string eject_color = ""; // "red" or "blue"

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(15.0, 0.01, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(9.0, 0.0, 35.0);          // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 25.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(50_ms, 4_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(50_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(50_ms, 4_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(50_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 60);
  chassis.slew_drive_constants_set(5_in, 60);
  chassis.slew_swing_constants_set(3_in, 60);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(10_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(20_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path
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
void auto_wp_left() {
  wdLock = false;
  descore.set(true);
  chassis.drive_angle_set(0);
  chassis.odom_xyt_set(0, 0, 0);
  chassis.pid_odom_set({{-8, 28}, fwd, 100});
  intakeUS.move(-127);
  intakeLS.move(127);
  chassis.pid_wait_until(2);
  matchload.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180, -100, 20, cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-7, 127);
  chassis.pid_wait_quick_chain();
  intakeUS.move(127);
  pros::delay(1000);
  chassis.pid_odom_set(20, 127);
  chassis.pid_wait_quick_chain();
  intakeUS.move(-127);
  chassis.pid_odom_set(9, DS/2);
  chassis.pid_wait_quick_chain();
  intakeLS.move(0);
  chassis.pid_odom_set(-5, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-135, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-52, 127);
  chassis.pid_wait_until(-35);
  intakeLS.move(-50);
  chassis.pid_wait_until(-42);
  trapdoor.set(true);
  intakeLS.move(127);
  chassis.pid_wait_quick_chain();
  pros::delay(700);
  chassis.pid_odom_set(5, 127);
  trapdoor.set(false);
  matchload.set(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 90, 100, -50, ccw);
  intakeUS.move(127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(50, DS);
  intakeUS.move(-127);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, -45, -90, -30);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(18, 127);
  chassis.pid_wait_until(8);
  intakeLS.move(-100);
  chassis.pid_wait_quick_chain();
  pros::delay(1500);
}

void auto_wp_right() {
  descore.set(true);
  chassis.drive_angle_set(90);
  chassis.odom_xyt_set(0, 0, 90);
  chassis.pid_odom_set(32, 127, true);
  chassis.pid_wait_until(10);
  matchload.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180, 127);
  chassis.pid_wait_quick_chain();
  intakeLS.move(127);
  intakeUS.move(-127);
  chassis.pid_odom_set(12, DS/1.5);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-26, 127, true);
  chassis.pid_wait_until(-6);
  matchload.set(false);
  chassis.pid_wait_until(-18);
  intakeUS.move(127);
  chassis.pid_wait_quick_chain();
  pros::delay(700);
  intakeUS.move(0);
  chassis.pid_swing_set(ez::LEFT_SWING, -88, 60, -127, cw);
  intakeUS.move(127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(45, 127); //44-46
  intakeUS.move(-127);
  chassis.pid_wait_until(39);
  matchload.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 175, 127, 52, ccw); //50-60
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-5, 127);
  chassis.pid_wait_quick_chain();
  intakeUS.move(127);
  chassis.pid_odom_set(-15, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-180, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(20, 127);
  intakeUS.move(-127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(9, DS/2);
  chassis.pid_wait_quick_chain();
  intakeLS.move(0);
  chassis.pid_odom_set(-5, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-135, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-50, 127, false);
  chassis.pid_wait_until(-35);
  intakeLS.move(-50);
  chassis.pid_wait_until(-40);
  trapdoor.set(true);
  intakeLS.move(127);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  chassis.pid_odom_set(5, 127);
  trapdoor.set(false);
  matchload.set(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 0, 127, 10, cw);
  chassis.pid_wait_quick_chain();
  descore.set(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 15, 127, 20, cw);
  chassis.pid_wait_quick_chain();
}

void left_sweep() {
  descore.set(true);
  chassis.drive_angle_set(0);
  chassis.odom_xyt_set(0, 0, 0);
  chassis.pid_odom_set({{-8, 28}, fwd, 100});
  intakeUS.move(-127);
  intakeLS.move(127);
  chassis.pid_wait_until(2);
  matchload.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180, -100, 20, cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-7, 127);
  chassis.pid_wait_quick_chain();
  intakeUS.move(127);
  pros::delay(1000);
  chassis.pid_odom_set(20, 127);
  chassis.pid_wait_quick_chain();
  intakeUS.move(-127);
  chassis.pid_odom_set(9, DS/2);
  chassis.pid_wait_quick_chain();
  intakeLS.move(0);
  chassis.pid_odom_set(-5, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-135, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-50, 127);
  chassis.pid_wait_until(-35);
  intakeLS.move(-50);
  chassis.pid_wait_until(-40);
  trapdoor.set(true);
  intakeLS.move(127);
  chassis.pid_wait_quick_chain();
  pros::delay(700);
  chassis.pid_odom_set(5, 127);
  trapdoor.set(false);
  matchload.set(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 0, 127, 10, cw);
  chassis.pid_wait_quick_chain();
  descore.set(false);
  chassis.pid_odom_set(5, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(20, 127);
  chassis.pid_wait_quick_chain();
}

void right_sweep() {
  descore.set(true);
  chassis.drive_angle_set(90);
  chassis.odom_xyt_set(0, 0, 90);
  chassis.pid_odom_set(32, 127, true);
  chassis.pid_wait_until(10);
  matchload.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180, 127);
  chassis.pid_wait_quick_chain();
  intakeLS.move(127);
  intakeUS.move(-127);
  chassis.pid_odom_set(12, DS/2);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-26, 127, true);
  chassis.pid_wait_until(-6);
  matchload.set(false);
  chassis.pid_wait_until(-18);
  intakeUS.move(127);
  chassis.pid_wait_quick_chain();
  pros::delay(700);
  intakeUS.move(0);
  chassis.pid_swing_set(ez::LEFT_SWING, -85, 60, -127, cw);
  intakeUS.move(127);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, -45, -90, 30, cw);
  intakeUS.move(-127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(10, 127);
  chassis.pid_wait_until(5);
  intakeLS.move(-127);
  chassis.pid_wait_quick_chain();
  pros::delay(1000);
  chassis.pid_swing_set(ez::LEFT_SWING, -180, -127, 20, ccw);
  chassis.pid_wait_quick_chain();
  descore.set(false);
  chassis.pid_odom_set(-5, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(160, 127);
  chassis.pid_wait_quick_chain();
}

void left_contest() {
  descore.set(true);
  chassis.drive_angle_set(-90);
  chassis.odom_xyt_set(0, 0, -90);
  chassis.pid_odom_set(29, 127, true);
  chassis.pid_wait_until(10);
  matchload.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-180, 127);
  chassis.pid_wait_quick_chain();
  intakeLS.move(127);
  intakeUS.move(-127);
  chassis.pid_odom_set(12, DS/3);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-25, DS, true);
  chassis.pid_wait_until(-6);
  matchload.set(false);
  chassis.pid_wait_quick_chain();
  intakeUS.move(127);
  pros::delay(900);
  intakeUS.move(0);
  chassis.pid_odom_set(1, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 0, 127, -20, ccw);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(18, 127);
  chassis.pid_wait_until(7);
  descore.set(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(20, TS);
  chassis.pid_wait_quick_chain();
}

void right_contest() {
  descore.set(true);
  chassis.drive_angle_set(90);
  chassis.odom_xyt_set(0, 0, 90);
  chassis.pid_odom_set(29, 127, true);
  chassis.pid_wait_until(10);
  matchload.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180, 127);
  chassis.pid_wait_quick_chain();
  intakeLS.move(127);
  intakeUS.move(-127);
  chassis.pid_odom_set(10, DS/3);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-25, DS, true);
  chassis.pid_wait_until(-6);
  matchload.set(false);
  chassis.pid_wait_quick_chain();
  intakeUS.move(127);
  pros::delay(900);
  intakeUS.move(0);
  chassis.pid_odom_set(1, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 0, 127, -20, ccw);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(18, 127);
  chassis.pid_wait_until(7);
  descore.set(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(20, TS);
  chassis.pid_wait_quick_chain();
}

void skills() {
  wdLock = false;
  descore.set(true);
  chassis.drive_angle_set(0);
  chassis.odom_xyt_set(0, 0, 0);
  intakeUS.move(-127);
  intakeLS.move(127);
  chassis.pid_odom_set({{-15, 15, -45}, fwd, 127});
  chassis.pid_wait_quick_chain();
  matchload.set(true);
  chassis.pid_odom_set(5, DS/3);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-15, 32, -135}, rev, 127});
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-7, DS/2);
  intakeLS.move(-50);
  chassis.pid_wait_until(-5);
  trapdoor.set(true);
  chassis.pid_wait_quick_chain();
  intakeLS.move(127);
  pros::delay(2000);
  trapdoor.set(false);
  chassis.pid_odom_set(25, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-47, 0, -180}, fwd, 127});
  intakeLS.move(0);
  chassis.pid_wait_quick_chain();
  intakeLS.move(127);
  chassis.pid_odom_set(10, DS/3);
  chassis.pid_wait_quick_chain();
  pros::delay(1000);
  chassis.pid_odom_set(-10, 127);
  chassis.pid_wait_quick_chain();
  intakeLS.move(0);
  intakeUS.move(0);
  chassis.pid_turn_set(-225, TS);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-15, 127);
  chassis.pid_wait_quick_chain();
  matchload.set(false);
  chassis.pid_turn_set(180, TS);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-49, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 0, -127, 3);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-9, DS);
  intakeLS.move(127);
  intakeUS.move(127);
  pros::delay(2000);
  intakeUS.move(0);
  chassis.pid_odom_set(15, DS/1.5);
  matchload.set(true);
  intakeUS.move(-127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(11, DS/3);
  chassis.pid_wait_quick_chain();
  pros::delay(1500);
  chassis.pid_odom_set(-24.5, DS, true);
  chassis.pid_wait_until(-6);
  matchload.set(false);
  chassis.pid_wait_quick_chain();
  intakeUS.move(127);
  pros::delay(2000);
  intakeUS.move(0);
  chassis.pid_odom_set(5, DS);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-18, 63, 135}, fwd, 60});
  intakeUS.move(-127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(7, DS/3);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  intakeLS.move(-80);
  pros::delay(2000);
  chassis.pid_odom_set(-5, DS);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{ 35, 65, 90}, fwd, 60});
  intakeLS.move(127);
  intakeUS.move(-127);
  chassis.pid_wait_quick_chain();
  matchload.set(true);
  chassis.pid_drive_set(-9, DS/2);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(45, TS);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-14, DS/2);
  chassis.pid_wait_until(-12);
  trapdoor.set(true);
  chassis.pid_wait_quick_chain();
  intakeLS.move(127);
  pros::delay(2000);
  chassis.pid_odom_set(25, 127);
  chassis.pid_wait_quick_chain();
  trapdoor.set(false);
  chassis.pid_odom_set({{47, 100, 0}, fwd, DS});
  chassis.pid_wait_quick_chain();
  pros::delay(1000);
  chassis.pid_odom_set(-10, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-45, TS);
  intakeLS.move(0);
  intakeUS.move(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-15, 127);
  chassis.pid_wait_quick_chain();
  matchload.set(false);
  chassis.pid_turn_set(0, TS);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-50, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::RIGHT_SWING, 180, -127, 3);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(-10, DS);
  intakeLS.move(127);
  intakeUS.move(127);
  pros::delay(2000);
  intakeUS.move(0);
  chassis.pid_odom_set(15, DS/1.5);
  matchload.set(true);
  intakeUS.move(-127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(11, DS/3);
  chassis.pid_wait_quick_chain();
  pros::delay(1500);
  chassis.pid_odom_set(-24.5, DS, true);
  chassis.pid_wait_until(-6);
  chassis.pid_wait_quick_chain();
  intakeUS.move(127);
  pros::delay(2000);
  intakeUS.move(0);
  matchload.set(false);
  chassis.pid_odom_set(5, DS);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{24, 24, -45}, fwd, DS});
  intakeUS.move(127);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set(9, DS/2);
  intakeUS.move(-127);
  chassis.pid_wait_quick_chain();
  intakeLS.move(-80);
  pros::delay(2000);
  chassis.pid_odom_set(-2, DS);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{3, 10, 180}, fwd, DS});
  intakeLS.move(127);
  intakeUS.move(127);
  chassis.pid_wait_quick_chain();
  aligner.set(true);
  chassis.pid_drive_set(40, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20, 127);
  chassis.pid_wait_quick_chain();
}