#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/optical.hpp"
#include <algorithm>
#include <ctime>
#include <time.h>
#include <type_traits>
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
#include <chrono>



int readyscoreposition = 0;
int normalposition = 1;
int loadposition = 0;
int mobileflippernum = 1;
int intakeraisernum = 1;
int doinkernum = 1;



int autointakespeed = 12700;
int prevdistance = 0;
int target = 0;
// controller
// controller
pros::Controller master (CONTROLLER_MASTER);
pros::Motor intake ({18}, pros::MotorGearset::blue);
pros::MotorGroup armmotor ({-17, 21}, pros::MotorGearset::green);

// motor groupss
pros::MotorGroup left_motors({-1, -2, 3}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup right_motors({11, 12, -13}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges

// Inertial Sensor on port 10
pros::Imu imu(8);
pros::Rotation armrotationsensor(16);


pros::Optical colorSortSensor(7);



pros::adi::DigitalOut mobilegoalmech('A');
pros::adi::DigitalOut doinker('B');
pros::adi::DigitalOut intakeraiser('D');
pros::adi::DigitalOut mobileflipper('C');


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 12 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            0, // derivative gain (kD)
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

































































int distance = 0;



void LB(){
    
    while (true){
        if (target == 9300)
            loadposition = 1;
        else
            loadposition = 0;

        float kP= 0.008;
        float kD= 0.012;
        distance = target - armrotationsensor.get_angle();
        int derivative = distance - prevdistance;
        int armmovespeed = distance*kP+kD*derivative;
        armmotor.move_velocity(armmovespeed);
        int prevdistance = distance;
        pros::delay(5);

    }

}








unsigned long start_intake_timer; 
unsigned long end_intake_timer; 
unsigned long start_autointake_timer; 
unsigned long end_autointake_timer; 

bool auctally_sort_red = true;
bool do_sort_red;

bool sort_red = false;
bool is_red = true;
int Initial_delay;

void Match_Sort(){
     if(intake.get_actual_velocity()>140){
     Initial_delay = 7;
     }
     else if(intake.get_actual_velocity()<140){
     Initial_delay = 10;
     }
     Initial_delay = 980/intake.get_actual_velocity()/(2)-7;
    pros::delay(95); //Delay to tune break point

    intake.move_voltage(12000);
}










int runautointake;


void AutoIntake(){
    int runtimer = false;
    start_autointake_timer = end_autointake_timer = clock();
    do_sort_red = false;
    while (true){
        end_autointake_timer = clock();
        double delta_time = (double)(end_autointake_timer - start_autointake_timer)/CLOCKS_PER_SEC;

        if (delta_time < 0.0) start_autointake_timer = end_autointake_timer;


        //master.set_text(2, 0, "Color Match!");

        is_red = true;
        //If button R1 is being pressed, spin teh intake forwards at full speed
        if (runautointake == 1) {
        intake.move_voltage(autointakespeed);
        pros::delay(5);



        if (delta_time > 0.01){

            if ((intake.get_actual_velocity() < 5) && (loadposition == 0)){

                    intake.move_voltage(-12000);
                    pros::delay(200); //Delay to control length of break period
                    intake.move_voltage(12000);
                    start_autointake_timer = clock();
                    }
            
            }
        }
        else{
            start_autointake_timer = end_autointake_timer  = clock();
            intake.move_voltage(0);

                }
        //If button "Y" is pressed: Sets intake to sort opposite color of the previous sort color
         //if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
         //    do_sort_red = !do_sort_red;   
         //}
         int Hue = colorSortSensor.get_hue();
         if (do_sort_red == true)
            master.set_text(2, 0, "Sorting RED!");
         if (do_sort_red == false)
            master.set_text(2, 0, "Sorting BLUE!");

             if ((Hue<40)){
                 is_red = true;
                 if (do_sort_red == true){
                    if (auctally_sort_red == true){
                        intake.move_voltage(-12000);
                        pros::delay(50); //Delay to control length of break period
                    }else {
                        intake.move_voltage(-5000);
                        pros::delay(10); //Delay to control length of break period
                        runautointake = 0;
                    }
                }
             }
             if ((Hue < 260) & (Hue > 205)){
                 is_red = false;
                 if (do_sort_red == false){
                    intake.move_voltage(-12000);
                    pros::delay(50); //Delay to control length of break period
                    }

             }
             //if(is_red == false){
             //    Match_Sort();  
             //    }   
             

            // if (target_position > 0 & target_position < 6){
            //     if (IntakeMotor.get)
            // }

    // printf("my int: %d\n", Initial_delay);

    pros::delay(5);
    }
}



















void Intake(){
    int runtimer = false;
    start_intake_timer = clock();
    end_intake_timer = clock();
    while (true){
        end_intake_timer = clock();
        double delta_time = (double)(end_intake_timer - start_intake_timer)/CLOCKS_PER_SEC;

        if (delta_time < 0) start_intake_timer = end_intake_timer;



        
        
        
        //master.set_text(2, 0, "Color Match!");

        is_red = true;
        //If button R1 is being pressed, spin teh intake forwards at full speed
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake.move_voltage(12700);



        if (delta_time > 0.02){

            if ((intake.get_actual_velocity() < .01) && (loadposition == 0)){

                    intake.move_voltage(-12000);
                    pros::delay(80); //Delay to control length of break period
                    intake.move_voltage(12000);
                    start_intake_timer = clock();
                    }
            
            }
        }
        else{
            start_intake_timer = clock();
            end_intake_timer = clock();
            intake.move_voltage(0);

                }
        //If button "Y" is pressed: Sets intake to sort opposite color of the previous sort color
         if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
             do_sort_red = !do_sort_red;   
         }

         if (do_sort_red == true)
            master.set_text(2, 0, "Sorting RED!");
         if (do_sort_red == false)
            master.set_text(2, 0, "Sorting BLUE!");

        if(colorSortSensor.get_proximity()<50){
          int Hue = colorSortSensor.get_hue();
        if ((Hue<20)){
            is_red = true;
            if (do_sort_red == true){
            intake.move_voltage(-12000);
                pros::delay(50); //Delay to control length of break period
        }
        }

        if ((Hue < 260) && (Hue > 205)){
            is_red = false;
            if (do_sort_red == false){
                intake.move_voltage(-12000);
                pros::delay(50); //Delay to control length of break period
            }

        }
        //if(is_red == false){
        //    Match_Sort();  
        //}   
    }
             

            // if (target_position > 0 & target_position < 6){
            //     if (IntakeMotor.get)
            // }

    // printf("my int: %d\n", Initial_delay);

    pros::delay(5);
    }
}

// Get color without delay
static int get_opticalColor() {
    double hue = colorSortSensor.get_hue();
    if (colorSortSensor.get_proximity() < 100) return 1; //none //IMPORTANT: was set to 100 for autons
    if (hue < 22 || hue > 355) return 2; //red
        master.set_text(2, 0, "Color Match!");

    if (hue > 200 && hue < 240) return 3; //blue
        master.set_text(2, 0, "Color No Match!");

    return 1;
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
    colorSortSensor.set_integration_time(1.0);
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
ASSET(Skillsauton2_txt);

// Create the decoder
lemlib_tarball::Decoder decoder(Skillsauton1_txt);
lemlib_tarball::Decoder decoder2(Skillsauton2_txt);


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
    chassis.setPose(-60, 0, 90);
    // Follow paths by their names from PATH.JERRYIO
    // Parameters: path, lookahead distance, timeout
  
    mobilegoalmech.set_value(true);
    runautointake = 1;
    loadposition = 1;
    pros::Task AutoIntakeStart(AutoIntake);
    pros::delay(500);
    runautointake = 0;
    loadposition = 0;

    //pros::Task AutoIntakeStart1(AutoIntake);
  
    chassis.follow(decoder["Path 1"], 15, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 500);
    chassis.waitUntilDone();
  
    chassis.follow(decoder["Path 2"], 15, 3000, false);
    chassis.waitUntilDone();
    mobilegoalmech.set_value(false);
    pros::delay(500);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 500);
    chassis.waitUntilDone();
    runautointake = 1;
    //pros::Task AutoIntakeStart2(AutoIntake);
  
    chassis.follow(decoder["Path 3"], 15, 3000);
    chassis.waitUntilDone();
    chassis.turnToHeading(45, 500);
    chassis.waitUntilDone();
  
    chassis.follow(decoder["Path 4"], 15, 2000);
    chassis.waitUntilDone();
  
  
    target =9300;
    pros::Task AutonLB(LB);
  
    chassis.turnToHeading(0, 500);
    chassis.waitUntilDone();
  
  
  
    chassis.follow(decoder["Path 5"], 15, 2000);
  
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 500);
  
    chassis.waitUntilDone();
  
    pros::delay(500);
    runautointake = 0;
    //pros::Task AutoIntakeStart3(AutoIntake);
  
    target =22000;
  
    pros::delay(500);
  
    chassis.follow(decoder["Path 6"], 15, 4000, false);
    chassis.waitUntilDone();
  
    chassis.turnToHeading(270, 500);
    runautointake = 1;
    //pros::Task AutoIntakeStart4(AutoIntake);
    chassis.waitUntilDone();
  
    target =7200;
  
    chassis.follow(decoder["Path 7"], 15, 4000);
    chassis.waitUntilDone();
    pros::delay(500);
  
    chassis.follow(decoder["Path 7.5"], 15, 2000);
    chassis.waitUntilDone();
  
    chassis.follow(decoder["Path 7.75"], 15, 1000, false);
    chassis.waitUntilDone();
  
    chassis.setPose(-59, 48, 270);
    chassis.waitUntilDone();
    chassis.turnToHeading(45, 500, {.maxSpeed = 75});
  
    chassis.follow(decoder["Path 8"], 15, 3000);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 500, {.maxSpeed = 75});
    chassis.waitUntilDone();
  
    chassis.follow(decoder["Path 9"], 15, 2000, false);
    chassis.waitUntilDone();
    runautointake = 0;
    //pros::Task AutoIntakeStart5(AutoIntake);
    mobilegoalmech.set_value(true);
    chassis.follow(decoder["Path 10"], 15, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 500);
    chassis.waitUntilDone();
  
  
    chassis.follow(decoder["Path 11"], 15, 1000);
    chassis.waitUntilDone();
    pros::delay(500);
  
    chassis.setPose(-40, 59, 0);
    chassis.waitUntilDone();
  
    chassis.follow(decoder["Path 12"], 15, 2000, false);
    chassis.waitUntilDone();
    chassis.follow(decoder["Path 12.5"], 15, 2000, false);
    chassis.waitUntilDone();
  
    mobilegoalmech.set_value(false);
    pros::delay(500);
  
  
    chassis.turnToHeading(90, 500);
    chassis.waitUntilDone();
    runautointake = 1;
    //pros::Task AutoIntakeStart6(AutoIntake);
  
    chassis.follow(decoder["Path 13"], 15, 3000);
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 500);
    chassis.waitUntilDone();
  
    chassis.follow(decoder["Path 14"], 15, 2000);
    target =9300;
    chassis.waitUntilDone();
  
  
  
    chassis.turnToHeading(180, 500);
    chassis.waitUntilDone();
  
    chassis.follow(decoder["Path 15"], 15, 2000);
  
    chassis.waitUntilDone();
  
    pros::delay(750);
    runautointake = 0;
  
    target =22000;
  
    pros::delay(500);
  
    chassis.follow(decoder["Path 16"], 15, 4000, false);
    chassis.waitUntilDone();
  
    chassis.turnToHeading(270, 600);
    chassis.waitUntilDone();

    runautointake = 1;
    //pros::Task AutoIntakeStart8(AutoIntake);
  
    target =7200;
  
    chassis.follow(decoder["Path 17"], 15, 4000);
    pros::delay(500);
  
  
    chassis.follow(decoder["Path 17.5"], 15, 2000);
    chassis.waitUntilDone();
  
    chassis.follow(decoder["Path 17.75"], 15, 1000, false);
    chassis.waitUntilDone();
  
    chassis.setPose(-59, -48, 270);
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 500, {.maxSpeed = 75});
  
  
  
  
    chassis.follow(decoder["Path 18"], 15, 3000);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 500, {.maxSpeed = 75});
    chassis.waitUntilDone();
  
    chassis.follow(decoder["Path 19"], 15, 2000, false);
    chassis.waitUntilDone();
    runautointake = 0;
    //pros::Task AutoIntakeStart5(AutoIntake);
    mobilegoalmech.set_value(true);
    pros::delay(500);

    runautointake = 1;
    auctally_sort_red = false;
    do_sort_red = true;
    autointakespeed = 8000;
    chassis.follow(decoder["Path 20"], 15, 5000);
    chassis.waitUntilDone();
    chassis.turnToHeading(225, 1000);
    chassis.waitUntilDone();

    chassis.follow(decoder["Path 21"], 15, 1000, false);
    chassis.waitUntilDone();
    do_sort_red = false;
    mobilegoalmech.set_value(false);
    pros::delay(250);
    autointakespeed = 12700;
    runautointake = 1;
    chassis.turnToHeading(210, 500, {.maxSpeed = 75});
    chassis.waitUntilDone();

    chassis.follow(decoder["Path 22"], 15, 4000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 1500, {.maxSpeed = 75});
    chassis.waitUntilDone();


    chassis.follow(decoder["Path 22.5"], 15, 3000);
    chassis.waitUntilDone();

    chassis.turnToHeading(315, 1500);
    chassis.waitUntilDone();

    runautointake = 0;

    chassis.follow(decoder["Path 23"], 15, 3000);
    chassis.waitUntilDone();
    runautointake = 1;

    chassis.follow(decoder["Path 23.5"], 15, 3000);
    chassis.waitUntilDone();

    chassis.turnToHeading(45, 700);
    chassis.waitUntilDone();
    runautointake = 0;

    chassis.follow(decoder["Path 24"], 15, 3000);
    chassis.waitUntilDone();
    runautointake = 1;

    chassis.follow(decoder["Path 24.5"], 15, 3000);
    chassis.waitUntilDone();

    chassis.turnToHeading(0, 1500);
    chassis.waitUntilDone();

    chassis.follow(decoder["Path 25"], 15, 3000);
    chassis.waitUntilDone();
    chassis.turnToHeading(65, 1500);
    chassis.waitUntilDone();
    chassis.follow(decoder["Path 26"], 15, 3000);
    chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000);
    chassis.waitUntilDone();
    mobilegoalmech.set_value(true);
    chassis.follow(decoder["Path 27"], 15, 500, false);
    chassis.waitUntilDone();

    chassis.follow(decoder["Path 28"], 15, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(225, 1000);
    chassis.waitUntilDone();

    chassis.follow(decoder["Path 29"], 15, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 1000);
    chassis.waitUntilDone();
    chassis.follow(decoder["Path 30"], 15, 4000);
    chassis.waitUntilDone();

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
    
    loadposition = 1;

	

   
    pros::Task Sort(Intake);



	while (true) {




	// Check if the R1 button on the controller is pressed









	// get joystick positions
	int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
	// move the chassis with curvature drive
	chassis.tank(leftY, rightY);

	if (master.get_digital(DIGITAL_A))
    {
        intake.move_voltage(-12000);
    }




	if (master.get_digital(DIGITAL_R2))
		{
		mobilegoalmech.set_value(true);
		}
	else
		{
		mobilegoalmech.set_value(false);
		}
		
	
    if (master.get_digital_new_press(DIGITAL_DOWN))
    if (doinkernum == 1){
        doinker.set_value(true);
        doinkernum = 0;
      }
  else {
        doinker.set_value(false);
        doinkernum = 1;
  }


      



	if (master.get_digital_new_press(DIGITAL_X))
    if (mobileflippernum == 1){
		  mobileflipper.set_value(true);
      mobileflippernum = 0;
		}
    else {
		  mobileflipper.set_value(false);
      mobileflippernum = 1;
    }


	if (master.get_digital_new_press(DIGITAL_B))
    if (intakeraisernum == 1){
        intakeraiser.set_value(true);
          intakeraisernum = 0;
		}
    else {
        intakeraiser.set_value(false);
          intakeraisernum = 1;
    }




	if (master.get_digital(DIGITAL_L1)) {
			target =22000;
            readyscoreposition = 1;
            normalposition = 0; 
            loadposition = 0;
 
	}	
	else if (master.get_digital(DIGITAL_L2)) {
			target =9300;
            readyscoreposition = 0;
            normalposition = 1;
            loadposition = 1;
	}	
	else if (readyscoreposition == 1) {
			target =18000;

	}	
	else if (normalposition == 1) {
			target =7200;
            loadposition = 0;

	}


	float kP= 0.008;
	float kD= 0.012;
	int distance = target - armrotationsensor.get_angle();
	int derivative = distance - prevdistance;
	int armmovespeed = distance*kP+kD*derivative;
	armmotor.move_velocity(armmovespeed);
	int prevdistance = distance;
    pros::delay(2);
	
  }

}