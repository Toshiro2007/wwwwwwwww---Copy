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
#include "lemlib-tarball/api.hpp"

int readyscoreposition = 0;
int normalposition = 1;
int doinkernum = 1;
int intakeraisernum = 1;
int mobileflippernum = 1;



int prevdistance = 0;
int target = 0;


pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::adi::DigitalOut mobilegoalmech('A');
pros::adi::DigitalOut doinker('B');
pros::adi::DigitalOut intakeraiser('D');
pros::adi::DigitalOut mobileflipper('C');

// controller
// controller
pros::Motor intake ({19}, pros::MotorGearset::blue);
pros::Motor intakepreroller ({-18}, pros::MotorGearset::blue);
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






















































































bool sort_red = false;
bool is_red = true;

void Match_Sort(){
    master.set_text(2, 0, "Color Match ye!");

    // if(IntakeMotor.get_actual_velocity()>140){
    // Initial_delay = 7;
    // }
    // else if(IntakeMotor.get_actual_velocity()<140){
    // Initial_delay = 10;
    // }
    // Initial_delay = 980/IntakeMotor.get_actual_velocity()/(2)-7;
    pros::delay(65); //Delay to tune break point
    intake.move_voltage(-12000);
    pros::delay(100); //Delay to control length of break period
    intake.move_voltage(12000);
}

void Intake(){
    while (true){
        is_red = true;
        //If button R1 is being pressed, spin teh intake forwards at full speed
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move_voltage(12000);
        }
        else if (!(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))) {
        intake.move_voltage(0);
        }
        //If button "Y" is pressed: Sets intake to sort opposite color of the previous sort color
        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        //     sort_red = !sort_red;   
        // }

        //     if ((Ring_Optical.get_hue()<13)){
        //         is_red = true;
        //     }
        //     if ((Ring_Optical.get_hue() < 260) & (Ring_Optical.get_hue() > 215)){
        //         is_red = false;
        //     }
            // if(get_opticalColor() == 3){
            //     //Sort Blue
            //     if ((Ring_Distance.get() < 25)){
            //     target_position = 40;  
            //     } 
            //     if ((Ring_Distance.get() < 10)){
            //     Match_Sort();  
            //     }   
            // }

            // if (target_position > 0 & target_position < 6){
            //     if (IntakeMotor.get)
            // }

    // printf("my int: %d\n", Initial_delay);

    pros::delay(11);
    }
}

// Get color without delay
static int get_opticalColor() {
    double hue = colorSortSensor.get_hue();
    if (hue < 10 || hue > 355) return 2; //red
    if (hue > 200 && hue < 240) return 3; //blue
}





void Auton_StopIntake(){
    while (true){
        intake.move_voltage(12000);
        if (get_opticalColor() == 3){ 
               intake.brake();
               pros::delay(2500);
               return;
        } 
    }
}





























































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




    colorSortSensor.set_integration_time(3);
    colorSortSensor.set_led_pwm(100);




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

// Create the decoder
lemlib_tarball::Decoder decoder(Skillsauton1_txt);


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


  // Set initial robot pose (x, y, heading)
  chassis.setPose(-63, 0, 90);
  // Follow paths by their names from PATH.JERRYIO
  // Parameters: path, lookahead distance, timeout
  chassis.follow(decoder["Path 1"], 15, 20000);
  chassis.follow(decoder["Path 2"], 15, 20000);
  chassis.follow(decoder["Path 3"], 15, 20000);
  chassis.follow(decoder["Path 4"], 15, 20000);
  chassis.follow(decoder["Path 5"], 15, 20000);
  chassis.follow(decoder["Path 6"], 15, 20000);
  chassis.follow(decoder["Path 7"], 15, 20000);
  chassis.follow(decoder["Path 8"], 15, 20000);
  chassis.follow(decoder["Path 9"], 15, 20000);
  chassis.follow(decoder["Path 10"], 15, 20000);
  chassis.follow(decoder["Path 11"], 15, 20000);
  chassis.follow(decoder["Path 12"], 15, 20000);





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
pros::Task Sort(Intake);
	while (true) {
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
		
	
	if (master.get_digital_new_press(DIGITAL_UP))
    if (doinkernum == 1){
		  doinker.set_value(true);
      doinkernum = 0;
		}
    else {
		  doinker.set_value(false);
      doinkernum = 1;
    }


	if (master.get_digital_new_press(DIGITAL_X))
    if (intakeraisernum == 1){
		  intakeraiser.set_value(true);
      intakeraisernum = 0;
		}
    else {
		  intakeraiser.set_value(false);
      intakeraisernum = 1;
    }






	if (master.get_digital_new_press(DIGITAL_B))
    if (mobileflippernum == 1){
		  mobileflipper.set_value(true);
      mobileflippernum = 0;
		}
    else {
		  mobileflipper.set_value(false);
      mobileflippernum = 1;
    }




	if (master.get_digital(DIGITAL_L1)) {
			target =19600;
            readyscoreposition = 1;
            normalposition = 0;  
	}	
	else if (master.get_digital(DIGITAL_L2)) {
			target =7450;
            readyscoreposition = 0;
            normalposition = 1;
	}	
	else if (readyscoreposition == 1) {
			target =16000;
	}	
	else if (normalposition == 1) {
			target =4450;
	}


	float kP= 0.01;
	float kD= 0.03;
	int distance = target - armrotationsensor.get_angle();
	int derivative = distance - prevdistance;
	int armmovespeed = distance*kP+kD*derivative;
	armmotor.move_velocity(armmovespeed);
	int prevdistance = distance;
    pros::delay(2);
	
  }

}
