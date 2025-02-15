#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/optical.hpp"
#include <algorithm>
#include "lemlib-tarball/api.hpp"




int righttpistonnumber = 0;
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/apix.h"









bool reversing = false;    // Track if the motor is currently reversing
bool spin_up_grace = true; // Grace period flag to allow for "spooling", when the motor starts spinning the first time

int lefttpistonnumber = 0;
int mobilegoalnumber = 0;
int intakeraisernumber = 0;
int doinkernumber = 0;
int prevdistance = 0;
int target = 0;
// controller
// controller
pros::Controller master (CONTROLLER_MASTER);
pros::MotorGroup intake ({19, -18}, pros::MotorGearset::blue);
pros::Motor armmotorgroup (17, pros::MotorGearset::green);

// motor groupss
pros::MotorGroup left_motors({-1, -2, 3}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({11, 12, -13}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges

// Inertial Sensor on port 10
pros::Imu imu(8);
pros::Rotation armrotationsensor(16);


pros::Optical colorSortSensor(7);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-4);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-6);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, .591);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.75);



// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11, // 12 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
// create a v5 rotation sensor on port 4
pros::Rotation horizontal_encoder(6);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0);

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
	pros::lcd::set_text(1, "Hello PROS User!");

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

pros::Motor armmotorgroup(17, pros::MotorGearset::green);
pros::adi::DigitalOut mobilegoal('A');
mobilegoal.set_value(true);
intake.move(127);

int target = 12000;
int distance = 10000;
int prevdistance = distance;  // Initialize prevdistance before the loop

// Start loop to move the arm until it reaches the target
while (distance > 500) {  // Continue until distance is less than or equal to 10
    float kP = 0.02;
    float kD = 0.03;
    
    distance = target - armrotationsensor.get_angle();  // Calculate the current distance from target
    int derivative = distance - prevdistance;  // Calculate the difference in distance (derivative)

    int armmovespeed = distance * kP + kD * derivative;  // Calculate motor speed using PD control

    armmotorgroup.move_velocity(armmovespeed);  // Set motor speed to move the arm

    prevdistance = distance;  // Update prevdistance for next iteration

    pros::delay(2);  // Delay for motor response time
}



target =0;

distance = 100;{
while (distance < 10);			
	float kP= 0.02;
	float kD= 0.03;
	distance = target - armrotationsensor.get_angle();
	int derivative = distance - prevdistance;
	int armmovespeed = distance*kP+kD*derivative;
	armmotorgroup.move_velocity(armmovespeed);
	int prevdistance = distance;
	pros::delay(2);
}



// Once the arm is in position (distance <= 10), set the chassis pose

// Move the chassis to a specific point



chassis.moveToPoint(0, -11, 1000, {.forwards = false, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});
chassis.turnToHeading(90, 1000, {.maxSpeed = 50});
chassis.moveToPoint(-22, -11, 1000, {.forwards = false, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});
pros::delay(1000);  // Delay for motor response time
mobilegoal.set_value(false);
pros::delay(1000);  // Delay for motor response time

chassis.setPose(0, 0, 0);

chassis.turnToHeading(90, 1000, {.maxSpeed = 50});
chassis.setPose(0, 0, 0);
pros::delay(1000);  // Delay for motor response time

chassis.moveToPoint(20, 0, 1000, {.forwards = true, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});
pros::delay(1000);  // Delay for motor response time
chassis.setPose(0, 0, 0);
pros::delay(1000);  // Delay for motor response time


pros::delay(1000);  // Delay for motor response time

chassis.turnToHeading(90, 1000, {.maxSpeed = 50});

pros::delay(1000);  // Delay for motor response time

chassis.setPose(0, 0, 0);

pros::delay(1000);  // Delay for motor response time



chassis.moveToPoint(0, 16, 2000, {.forwards = true, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});

pros::delay(3000);  // Delay for motor response time

chassis.turnToHeading(90, 1000, {.maxSpeed = 50});


pros::delay(1000);  // Delay for motor response time
chassis.setPose(0, 0, 0);
pros::delay(1000);  // Delay for motor response time

chassis.moveToPoint(0, 34, 3000, {.forwards = true, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});
pros::delay(1000);  // Delay for motor response time
chassis.setPose(0, 0, 0);
pros::delay(1000);  // Delay for motor response time
chassis.setPose(0, 0, 0);


pros::delay(1000);  // Delay for motor response time
chassis.turnToHeading(135, 1000, {.maxSpeed = 50});

pros::delay(1000);  // Delay for motor response time
chassis.setPose(0, 0, 0);
pros::delay(2000);  // Delay for motor response time

mobilegoal.set_value(true);
pros::delay(2000);  // Delay for motor response time

chassis.moveToPoint(0, -16, 2000, {.forwards = false, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});

pros::delay(2000);  // Delay for motor response time

chassis.moveToPoint(0, 25, 3000, {.forwards = true, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});
pros::delay(4000);  // Delay for motor response time


chassis.setPose(0, 0, 0);
chassis.turnToHeading(180, 4000, {.maxSpeed = 50});

pros::delay(5000);  // Delay for motor response time
chassis.setPose(0, 0, 0);

chassis.moveToPoint(0, -37, 2000, {.forwards = false, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});
pros::delay(2000);  // Delay for motor response time
chassis.turnToHeading(90, 4000, {.maxSpeed = 50});
chassis.moveToPoint(0, -13, 2000, {.forwards = false, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});

pros::delay(2000);  // Delay for motor response time
mobilegoal.set_value(false);
pros::delay(4000);  // Delay for motor response time

chassis.setPose(0, 0, 0);
chassis.turnToHeading(180, 4000, {.maxSpeed = 50});
pros::delay(5000);  // Delay for motor response time

chassis.setPose(0, 0, 0);
chassis.moveToPoint(0, 20, 3000, {.forwards = true, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});
pros::delay(4000);  // Delay for motor response time
chassis.setPose(0, 0, 0);

chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
pros::delay(2000);  // Delay for motor response time
chassis.setPose(0, 0, 0);
chassis.moveToPoint(0, -12, 3000, {.forwards = false, .maxSpeed = 50, .minSpeed = 60, .earlyExitRange = 2});


}
/**



	pros::adi::DigitalOut mobilegoal('A');
    mobilegoal.set_value(true);

    chassis.setPose(51, -58, 90);
    chassis.moveToPoint(23.5, -58, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 60, .earlyExitRange = 2});
    chassis.turnToPoint(4, -48.5, 180, {.forwards = false});
    chassis.moveToPoint(4, -48.5, 1450, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntilDone();
    intake.move(127);
    mobilegoal.set_value(false);
    pros::delay(200); //delay for clamp
    chassis.turnToPoint(4.5, -22, 700, {.forwards=false});
    chassis.moveToPoint(4.5, -22, 1500, {.forwards=false});
    pros::delay(700);
    mobilegoal.set_value(true);
    pros::delay(200);
    chassis.moveToPoint(4.5, -30, 1500);
    chassis.waitUntilDone();
    chassis.turnToHeading(215, 1000);

    pros::lcd::print(4, "pure pursuit finished!");
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

	pros::lcd::print(4, "intake:%2f:",intake.get_actual_velocity());
	pros::lcd::print(0, "intake:%2f:",intake.get_actual_velocity());

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::adi::DigitalOut righttpiston('C');
	pros::adi::DigitalOut lefttpiston('B');
	pros::adi::DigitalOut mobilegoal('A');
	pros::adi::DigitalOut doinker('D');
	pros::adi::DigitalOut intakeraiser('E');

    

	
    pros::Task *intake_monitor_task = nullptr; // Pointer to the intake monitoring task

   

   	// Start the color sorting task


	while (true) {




	// Check if the R1 button on the controller is pressed
	if (master.get_digital(DIGITAL_R1))		
        intake.move(127);










	// get joystick positions
	int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
	// move the chassis with curvature drive
	chassis.tank(leftY, rightY);

	if (master.get_digital(DIGITAL_R2))
		{
		mobilegoal.set_value(true);
		mobilegoalnumber = 0;
		}
	else
		{
		mobilegoal.set_value(false);
		mobilegoalnumber = 1;
		}
		
	
	if (master.get_digital_new_press(DIGITAL_X)) {
		if (doinkernumber == 0)
			{
			doinker.set_value(true);
			doinkernumber = 1;
			}
		else
		 	{
			doinker.set_value(false);
			doinkernumber = 0;
			}
		}

	if (master.get_digital_new_press(DIGITAL_RIGHT)) {
		if (lefttpistonnumber == 0)
			{
			lefttpiston.set_value(true);
			lefttpistonnumber = 1;
			}
		else
		 	{
			lefttpiston.set_value(false);
			lefttpistonnumber = 0;
			}
		}
	if (master.get_digital_new_press(DIGITAL_B)) {
		if (righttpistonnumber == 0)
			{
			righttpiston.set_value(true);
			righttpistonnumber = 1;
			}
		else
		 	{
			righttpiston.set_value(false);
			righttpistonnumber = 0;
			}
		}
	if (master.get_digital_new_press(DIGITAL_UP)) {
		if (intakeraisernumber == 0)
			{
			intakeraiser.set_value(true);
			intakeraisernumber = 1;
			}
		else
		 	{
			intakeraiser.set_value(false);
			intakeraisernumber = 0;
			}
		}
	pros::lcd::print(2, "intake:%2f:",intake.get_actual_velocity());



	if (master.get_digital(DIGITAL_R1)) {
      intake.move_velocity(600); // This is 600 because it's a 600rpm motor
	}
	else if (master.get_digital(DIGITAL_A)) {
      intake.move_velocity(-600); // This is 600 because it's a 600rpm motor
	}
	else {
      intake.move_velocity(0);
	}




	if (master.get_digital(DIGITAL_L1)) {
			target =18700;
	}	
	else if (master.get_digital(DIGITAL_L2)) {
			target =6675;

	}	
	else if (master.get_digital(DIGITAL_DOWN)) {
			target =10000;
	}	
	else if (master.get_digital(DIGITAL_LEFT)) {
			target =17500;
	}	

	else{
			target =3900;
	}
	float kP= 0.02;
	float kD= 0.03;
	int distance = target - armrotationsensor.get_angle();
	int derivative = distance - prevdistance;
	int armmovespeed = distance*kP+kD*derivative;
	armmotorgroup.move_velocity(armmovespeed);
	int prevdistance = distance;
    pros::delay(2);
	
  }

}
