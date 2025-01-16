#include "main.h"
#include "pros/optical.hpp"
#include <algorithm>
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
pros::Motor intake (19, pros::MotorGearset::blue);
pros::MotorGroup armmotorgroup ({17, -18}, pros::MotorGearset::green);

// motor groups
pros::MotorGroup left_motors({-1, -2, 3}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({11, 12, -13}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges

// Inertial Sensor on port 10
pros::Imu imu(4);
pros::Rotation armrotationsensor(16);


pros::Optical colorSortSensor(7);




// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 12 inch track width
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
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to nullptr as we are using IMEs
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
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
    chassis.turnToHeading(90, 167, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
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
 * @brief Desired velocity for the intake motor in RPM.
 */
const int desired_velocity = 700;

/** 
 * @brief Threshold velocity below which the motor is considered stuck.
 * @details If the motor's velocity falls below this value, corrective action will be taken.
 */
const int velocity_threshold = 50;

/** 
 * @brief Degrees to reverse the intake motor when it is stuck.
 */
const int reverse_degrees = 3390;

/** 
 * @brief Speed for reversing the intake motor.
 */
const int reverse_speed = -600;

/**
 * @brief Task function that monitors the intake motor for stalls and takes corrective action.
 *
 * This task continuously checks the velocity of the intake motor. If the velocity falls below
 * the defined threshold while the motor is supposed to be running, it assumes the motor is stuck.
 * To resolve this, the motor is reversed for a certain number of degrees before resuming
 * normal operation.
 *
 * @param param Pointer to additional data passed to the task (not used here, can be nullptr).
 */

void intake_monitor_task_function(void *param)
{
    bool reversing = false;    // Track if the motor is currently reversing
    bool spin_up_grace = true; // Grace period flag to allow for "spooling", when the motor starts spinning the first time

    while (true)
    {
        // Get the current velocity of the intake motor
        double current_velocity = intake.get_actual_velocity();

        // Allow a grace period for spin-up after the motor starts
        if (spin_up_grace)
        {
            pros::delay(200);      // 200ms delay for spin-up
            spin_up_grace = false; // Disable grace period after initial delay
            continue;              // Skip the stuck check during grace period
        }

        // Check if the intake motor is stuck
        if (!reversing && abs(current_velocity) < velocity_threshold && intake.get_target_velocity() != 0)
        {
            // Log a message to the LCD for debugging purposes
            pros::lcd::print(0, "Intake stuck! Reversing...");
            pros::lcd::print(2, "Current velocity: %.2f", current_velocity);

            // Reverse the intake motor to resolve the stall
            reversing = true; // Set reversing flag to avoid repeated reversals
            intake.move_relative(-reverse_degrees, reverse_speed);

            // Wait for the reverse motion to complete
            while (abs(intake.get_actual_velocity()) > 1)
            {
                pros::delay(10);
            }

            // Resume normal intake operation
            intake.move_velocity(desired_velocity);
            reversing = false; // Reset the reversing flag
        }

        // Delay to reduce CPU usage of the task
        pros::delay(20);
    }
}





/**
 * @brief Enumeration of alliance colors.
 * Used to determine the robot's team color and apply related logic.
 */

enum AllianceColor
{
    RED,
    BLUE,
    UNKNOWN
};

/**
 * @brief Global variable representing the current alliance color.
 * Initialized to red by default, but can be changed to blue if needed.`
 * Suggest having your autonomous routines automatically set the ALLIANCE_COLOR
 */
AllianceColor ALLIANCE_COLOR = RED;




/**
 * @brief Detects the color using the colorSortSensor.
 *
 * Uses the hue reading to determine if the detected color is RED, BLUE, or UNKNOWN.
 *
 * @return AllianceColor The detected color as an AllianceColor enum value.
 */
AllianceColor detectColor()
{
    int hue = colorSortSensor.get_hue();

    if (hue >= 330 || hue <= 30)
    {
        return RED;
    }
    else if (hue >= 210 && hue <= 270)
    {
        return BLUE;
    }
    else
    {
        return UNKNOWN; // Define UNKNOWN in your enum if needed
    }
}


/**
 * @brief Task function to handle color sorting logic.
 *
 * This function continuously monitors objects detected by the color sorting sensor.
 * It determines whether the detected object matches the ALLIANCE_COLOR and
 * controls the intake motor to either allow or reject the object.
 * This function should be run as a separate task to avoid blocking the main loop.
 */
void colorSortTask(void* param) 
{
    // Constants for motor behavior during color sorting
    constexpr int TRAVEL_DELAY = 100; // Delay (ms) before stopping to eject
    constexpr int STOP_DELAY = 200;   // Delay (ms) to ensure ejection
    constexpr int INTAKE_SPEED = 100; // Default motor speed for intake

    while (true)
    {
        // Detect the current color of the object using the sensor
        AllianceColor detectedColor = detectColor();

        // Scenario 1: Detected color matches the alliance color
        if (detectedColor == ALLIANCE_COLOR)
        {
            // Display a message indicating a color match
            master.set_text(2, 0, "Color Match!");
            // Intake motor continues to operate normally
        }
        // Scenario 2: Detected color does not match the alliance color
        else if (detectedColor != UNKNOWN)
        {
            // Brief delay to allow the object to reach the eject position
            pros::delay(TRAVEL_DELAY);

            // Stop the intake motor momentarily to eject the object
            intake.move_velocity(0);

            // Display a message indicating a color mismatch
            master.set_text(2, 0, "Color Mismatch!");

            // Allow time for the object to be ejected via inertia
            pros::delay(STOP_DELAY);

            // Resume normal intake motor operation
            intake.move_velocity(INTAKE_SPEED);
        }
        // Scenario 3: No object is detected by the sensor
        else
        {
            // Display a message indicating that no object is detected
            master.set_text(2, 0, "No Ring!");
            // No changes to the intake motor state; remains under external control
        }

        // Add a small delay to prevent excessive sensor polling or spamming messages
        pros::delay(50);
    }
}







/**
 * @brief Main operator control function.
 */
void opcontrol() {

	pros::lcd::print(4, "intake:%2f:",intake.get_actual_velocity());
	pros::lcd::print(0, "intake:%2f:",intake.get_actual_velocity());

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, 2, -4});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({3, -5, 6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	pros::adi::DigitalOut righttpiston('C');
	pros::adi::DigitalOut lefttpiston('B');
	pros::adi::DigitalOut mobilegoal('A');
	pros::adi::DigitalOut doinker('D');
	pros::adi::DigitalOut intakeraiser('E');

    

	
    pros::Task *intake_monitor_task = nullptr; // Pointer to the intake monitoring task

   

   	// Start the color sorting task

    pros::Task colorSort(colorSortTask, nullptr, "Intake Monitor Task");

	while (true) {




	// Check if the R1 button on the controller is pressed
	if (master.get_digital(DIGITAL_R1))
	{
		// Run the intake motor at the desired velocity
		intake.move_velocity(desired_velocity);

		// Start the intake monitoring task if not already running
		if (intake_monitor_task == nullptr)
		{
			intake_monitor_task = new pros::Task(intake_monitor_task_function, nullptr, "Intake Monitor Task");
		}
	}
	else
	{
		// Stop the intake motor
		intake.move_velocity(0);

		// Stop and destroy the intake monitoring task if running
		if (intake_monitor_task != nullptr)
		{
			intake_monitor_task->remove(); // Stop the task
			delete intake_monitor_task;    // Free the allocated memory
			intake_monitor_task = nullptr;
		}
	}













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
			target =19300;
	}	
	else if (master.get_digital(DIGITAL_L2)) {
			target =7142;

	}	
	else if (master.get_digital(DIGITAL_DOWN)) {
			target =10000;
	}	
	else if (master.get_digital(DIGITAL_LEFT)) {
			target =18500;
	}	

	else{
			target =4400;
	}
	float kP= 0.03;
	float kD= 0.04;
	int distance = target - armrotationsensor.get_angle();
	int derivative = distance - prevdistance;
	int armmovespeed = distance*kP+kD*derivative;
	armmotorgroup.move_velocity(armmovespeed);
	int prevdistance = distance;
    pros::delay(2);
	
  }

}
