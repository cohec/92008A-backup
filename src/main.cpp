#include "main.h"
#include <string>
#include "EZ-Template/util.hpp"
#include "liblvgl/extra/widgets/colorwheel/lv_colorwheel.h"
#include "liblvgl/misc/lv_color.h"
#include "pros/abstract_motor.hpp"
#include "pros/colors.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.h"
#include "pros/rtos.h"
#include "subsystems.hpp"
#include "autons.hpp"
#include "pros/colors.h"
#include "pros/colors.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
  // These are your drive motors, the first motor is used for sensing!
  {-11,-12,13},     // Left Chassis Ports (negative port will reverse it!)
  {20,19,-18},  // Right Chassis Ports (negative port will reverse it!)
    
  7,      // IMU Port
  2.75,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
  600);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
ez::tracking_wheel horiz_tracker(15, 2, 0.5);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(16, 2, 1.25);   // This tracking wheel is parallel to the drive wheels
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  lb.tare_position();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_right_set(&vert_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(1);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(5, 10);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    {"BLUE LEFT SIDE", autonomous_blue_left},
    {"BLUE RIGHT SIDE", autonomous_blue_right},
    {"RED LEFT SIDE", autonomous_red_left},
    {"RED RIGHT SIDE", autonomous_red_right},
    {"SKILLS", autonomous_skills},
    {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  lb.set_brake_mode(MOTOR_BRAKE_COAST);
  int nvt = 0; //no velocity timer
  bool reverse = false;
  int stage = 0;
  bool last_state = false;
  

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();
    chassis.opcontrol_arcade_standard(ez::SINGLE);



    // Slowmode
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
      chassis.opcontrol_speed_max_set(127 * 0.5);
    } else {
      chassis.drive_brake_set(MOTOR_BRAKE_COAST);
      chassis.opcontrol_speed_max_set(127);
    }
    
    // Pneumatics
    // Clamp change to toggle
    goalClamp.set(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
    
    // Intake lift
    intakeLift.set(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y));

    // Climb arm
    climbArm.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP));

    // Mini arm aka doinker moment
    ringrush.set(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2));
    goalrush.set(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

    // Intake
    bool xp = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool bp = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    if (xp || bp) {
      int direction = xp ? 1 : -1;
      if (hook.get_actual_velocity() == 0) {
        if (!reverse) {
          nvt = pros::millis();
          reverse = true;
        } else if (pros::millis() - nvt >= 500) {
          hook.move(-127 * direction);
        }
      } else {
        reverse = false;
        intake.move(127 * direction);
      }
    } else {
      intake.move(0);
      reverse = false;
    }

    // LB
    /*manual mode where 2 buttons control lift manually
    lb.get_encoder_units() 5degrees set state 1
    10degrees state 2
    150 degrees state 3
    while button is held, deactivate manual mode and switch to cycle mode
    cycle through these states*/
    int lbspeed = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    lb.set_brake_mode(MOTOR_BRAKE_HOLD);
    if (lbspeed > 0) {
      lb.move(lbspeed);
    } else if (lbspeed < 0) {
      lb.move(lbspeed);
    } else {
      lb.move(0);
    }
    // LB stages
    bool current_state = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    lb.set_brake_mode(MOTOR_BRAKE_COAST);
    if (current_state && !last_state) {
      if (stage == 0) {
        lbPID.target_set(50);
        stage++;
      } else if (stage == 1) {
        lbPID.target_set(220);
        stage++;
      } else if (stage == 2) {
        lbPID.target_set(0);
        stage = 0;
      }
      lb.move(lbPID.compute(lb.get_position()));
      last_state = current_state;
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}