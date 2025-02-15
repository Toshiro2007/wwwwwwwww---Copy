#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/optical.hpp"
#include <algorithm>
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/apix.h"


int readyscoreposition = 0;
int normalposition = 1;





int prevdistance = 0;
int target = 0;
// controller
// controller
pros::Controller master (CONTROLLER_MASTER);
pros::MotorGroup intake ({19, -18}, pros::MotorGearset::blue);
pros::MotorGroup intakepreroller ({19, -18}, pros::MotorGearset::blue);
pros::Motor armmotor (17, pros::MotorGearset::green);

// motor groupss
pros::MotorGroup left_motors({-1, -2, 3}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({11, 12, -13}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges

// Inertial Sensor on port 10
pros::Imu imu(8);
pros::Rotation armrotationsensor(16);


pros::Optical colorSortSensor(7);




// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11, // 12 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate();

	pros::lcd::initialize();

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy




// Replace my_paths.txt with your actual filename
// "." is replaced with "_" to overcome c++ limitations
ASSET(Skillsauton1_txt);




/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
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
	pros::adi::DigitalOut mobilegoalmech('A');

  // Set initial robot pose (x, y, heading), start intake, and release mobile goal
  chassis.setPose(-60, 0, 90);
  intake.move(127);
  mobilegoalmech.set_value(true);

  pros::delay(500);

  //Move to center and stop second stage
  chassis.moveToPose(-48, 0, 180, 10000, {.maxSpeed = 75});
  intake.move(0);
  pros::delay(500);
  
  //Move to Goal 1
  chassis.moveToPose(-48, 24, 180, 10000, {.forwards = false});
  pros::delay(4000);
  mobilegoalmech.set_value(false);
  pros::delay(2000);

  //Clamp and turn to face ring 1
  chassis.turnToHeading(90, 10000, {.maxSpeed = 75});

  pros::delay(500);

  //start intake and pickup ring 1
  intake.move(127);
  chassis.moveToPose(-24, 24, 45, 10000, {.forwards = true, .maxSpeed = 75});
  pros::delay(500);


  //go to location to not hit ladder when picking up ring 2
  chassis.moveToPose(0, 48, 45, 10000, {.maxSpeed = 75});
  pros::delay(500);


  //pickup ring 2
  chassis.moveToPose(24, 48, 45, 10000, {.maxSpeed = 75});
  pros::delay(500);


  //turn to face ring 3, 4, and 5
  chassis.turnToHeading(270, 10000, {.maxSpeed = 75});
  pros::delay(500);



  //pickup ring 3, 4, and 5
  chassis.moveToPose(-58, 48, 270, 10000, {.maxSpeed = 50});
  pros::delay(500);


  //wall reset
  chassis.moveToPose(-64, 48, 270, 10000, {.maxSpeed = 75});
  pros::delay(500);
  chassis.setPose(-62.5, 48, 270);
  pros::delay(500);


  //back up off wall
  chassis.moveToPose(-58, 48, 270, 10000, {.forwards = false, .maxSpeed = 75});
  pros::delay(500);


  //turn to align to corner
  chassis.turnToHeading(45, 10000, {.maxSpeed = 75});
  pros::delay(500);

  //align to corner
  chassis.moveToPose(-48, 59, 270, 10000, {.forwards = true, .maxSpeed = 75});
  pros::delay(500);


  //move into corner
  chassis.moveToPose(-57, 59, 270, 10000, {.forwards = false, .maxSpeed = 75});
  pros::delay(500);


  //drop goal and reverse second stage of intake slowly
  mobilegoalmech.set_value(true);
  intake.move(-21);
  pros::delay(500);

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








/**
 * @brief Main operator control function.
 */
void opcontrol() {


	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::adi::DigitalOut mobilegoalmech('A');
	pros::adi::DigitalOut doinker('D');
	pros::adi::DigitalOut intakeraiser('E');

    

	

   



	while (true) {




	// Check if the R1 button on the controller is pressed









	// get joystick positions
	int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
	// move the chassis with curvature drive
	chassis.tank(leftY, rightY);

    if (master.get_digital(DIGITAL_R1))	{
        intake.move(127);
        intakepreroller.move(127);
    }

    else if (master.get_digital(DIGITAL_A)) {
            intake.move(-127);
        intakepreroller.move(-127);

    }
    else
		{
        intake.move(0);
        intakepreroller.move(0);
		}






	if (master.get_digital(DIGITAL_R2))
		{
		mobilegoalmech.set_value(true);
		}
	else
		{
		mobilegoalmech.set_value(false);
		}
		
	






	if (master.get_digital(DIGITAL_L1)) {
			target =19600;
            readyscoreposition = 1;
            normalposition = 0;  
	}	
	else if (master.get_digital(DIGITAL_L2)) {
			target =7200;
            readyscoreposition = 0;
            normalposition = 1;
	}	
	else if (readyscoreposition == 1) {
			target =18500;
	}	
	else if (normalposition == 1) {
			target =4300;
	}


	float kP= 0.01;
	float kD= 0.04;
	int distance = target - armrotationsensor.get_angle();
	int derivative = distance - prevdistance;
	int armmovespeed = distance*kP+kD*derivative;
	armmotor.move_velocity(armmovespeed);
	int prevdistance = distance;
    pros::delay(2);
	
  }

}
