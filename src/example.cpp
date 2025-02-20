
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



void initialize() {
	chassis.calibrate()
	pros::lcd::initialize();
	pros::lcd::register_btn1_cb(on_center_button);
    colorSortSensor.set_integration_time(3);
    colorSortSensor.set_led_pwm(100);




}



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
    }
}