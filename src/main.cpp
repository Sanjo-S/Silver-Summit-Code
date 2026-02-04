#include "lemlib/api.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/rtos.h"
#include "main.h"




/**
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/




// PID Sensors
pros::Imu imu(12); // imu on port 12
pros::Rotation v_tracker(-17); //vertical rotational sensor on port 17 to track y
pros::Rotation h_tracker(7); // horizontal rotational sensor on port 7 to track x
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&h_tracker, lemlib::Omniwheel::NEW_2, -0.221);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&v_tracker, lemlib::Omniwheel::NEW_2, 1.586);




// Motors
pros::MotorGroup left_motors({-1, -2, -3}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({20, 19, 18}, pros::MotorGearset::blue); // right motors on ports 20, 19, 18
pros::MotorGroup intake({9, 10, 8}); // intake motors on ports 9, 10, and 8
pros::MotorGroup firstStage({9, 10}, pros::MotorGearset::blue); // firststage motor on ports 9 and 10
pros::Motor hood(8, pros::MotorGearset::blue); // hood motor on port 8




// Pistons
pros::ADIDigitalOut littleWil('C', false); // piston on special port C
pros::ADIDigitalOut Wing('B', false); // piston on special port B
pros::ADIDigitalOut hoodPiston('A', true); // piston on special port A




// Variables
bool littleWil_state = false; // tracks the state of the little wil piston
bool Wing_state = false; // tracks the state of the Wing piston




pros::Controller controller(pros::E_CONTROLLER_MASTER);




// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.05, // 11.05 inch track width
                              lemlib::Omniwheel::OLD_275, // using old 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);




// lateral PID controller
lemlib::ControllerSettings linearController(
                           15.5, // proportional gain (kP)
                           0.5, // integral gain (kI)
                           77, // derivative gain (kD)
                           3, // anti windup
                           1, // small error range, in inches
                           100, // small error range timeout, in milliseconds
                           3, // large error range, in inches
                           500, // large error range timeout, in milliseconds
                           0 // maximum acceleration (slew)
);




// angular PID controller
lemlib::ControllerSettings angularController(
                           8, // proportional gain (kP)
                           0, // integral gain (kI)
                           52, // derivative gain (kD)
                           3, // anti windup
                           1, // small error range, in degrees
                           100, // small error range timeout, in milliseconds
                           3, // large error range, in degrees
                           500, // large error range timeout, in milliseconds 500
                           0 // maximum acceleration (slew)
);




// sensors for odometry
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel (y-axis)
                            nullptr, // set to nullptr as we don't have a second vertical tracking wheel
                            &horizontal_tracking_wheel, // horizontal tracking wheel (x-axis)
                            nullptr, // set to nullptr as we don't have a second horizontal tracking wheel
                            &imu // inertial sensor
);




// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);




void initialize() {
    pros::lcd::initialize(); // check if removing affects anything
    chassis.calibrate(); // calibrate sensors
    vertical_tracking_wheel.reset();
    horizontal_tracking_wheel.reset();
    // set drive motors to coast mode
    for (int port : {1, 2, 3, 18, 19, 20}) { // check if changing to correct ports affect auton
        pros::Motor motor(port);
        motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    // print position to brain screen
    pros::Task screen_task([&]() {
    while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        // log position telemetry
        lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        // delay to save resources
        pros::delay(50);
    }
    });
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




void setLittleWil(bool state) {
    littleWil.set_value(state);
}




void setIntake(int speed) {
    hoodPiston.set_value(true);
    intake.move_velocity(speed);
}




void holdIntake() {
    hoodPiston.set_value(false);
    intake.move_velocity(600);
}




void setWing(bool state) {
    Wing.set_value(state);
}




void scoreMiddle() {
    hoodPiston.set_value(false); // Set the hood piston to close
    firstStage.move_velocity(600);
    hood.move_velocity(-200);
}




void auton_swap() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 45, 0, 1500, {.forwards = true, .lead = 0}, false); //initial forward movement
    chassis.turnToHeading(-90, 1200);
    setLittleWil(true);
    holdIntake();
    chassis.moveToPose(-8, 45, -90, 500, {.forwards = true, .lead = 0}, false);
    chassis.swingToHeading(-100, DriveSide::LEFT, 100); //swings to -100 deg, (locks left side)
    chassis.swingToHeading(-80, DriveSide::RIGHT, 100); //swings to -80 deg, (locks right side)
    chassis.swingToHeading(-90, DriveSide::LEFT, 100); //swings to -90 deg, (locks left side)
    chassis.moveToPose(-15, 45, -90, 1000, {.forwards = true, .lead = 0, .maxSpeed = 100}, false);
    pros::delay(350);
    //matchload
    chassis.moveToPoint(15, 47, 1000, {.forwards = false}, false);
    setIntake(600);
    chassis.moveToPoint(23, 47, 2000, {.forwards = false, .maxSpeed = 60}, true);
    setIntake(-600);
    pros::delay(100);
    setIntake(600);
    pros::delay(1450);
    pros::delay(300);
    chassis.moveToPose(6, 47, 135, 1300, {.forwards = true, .lead = 0, .maxSpeed = 90}, false); //move out after scoring
    setLittleWil(false);
    //long goal score
    holdIntake();
    chassis.moveToPose(30, 22, 138, 1700, {.forwards = true, .lead = .5}, false);
    chassis.turnToHeading(-45, 1500);
    chassis.moveToPoint(42, 3, 1700, {.forwards = false, .maxSpeed = 60}, false);
    setIntake(0); //temp
    setIntake(-600);
    pros::delay(100);
    scoreMiddle();
    pros::delay(100);
    setIntake(-600);
    pros::delay(100);
    scoreMiddle();
    pros::delay(1000);




    //middle score
    chassis.moveToPose(27, 16, -48, 1100, {.forwards = true, .lead = 0}, false);
    chassis.turnToHeading(-180, 500);
    chassis.moveToPose(28, -25, -180, 1500, {.forwards = true, .lead = .4}, false);
    holdIntake();
    //grab next blocks
    chassis.moveToPoint(7, -51, 2500, {.forwards = true}, false);
    chassis.turnToHeading(-90, 500);
    chassis.moveToPoint(23, -51, 2000, {.forwards = false, .maxSpeed = 60}, false);
    pros::delay(200);
    pros::delay(600);
    setIntake(600);
    pros::delay(100);
    setIntake(-600);
    pros::delay(100);
    setIntake(600);
    pros::delay(1450);
    pros::delay(1000);
    setIntake(0);
    holdIntake();


    //long goal




}




void auton_red_left() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 33, 0, 1200, {.forwards = true, .lead = 0}, false); //initial forward movement
    chassis.turnToHeading(-90, 700);
    setLittleWil(true);
    holdIntake();
    chassis.moveToPose(-8, 33, -90, 500, {.forwards = true, .lead = 0}, false);
    chassis.swingToHeading(-100, DriveSide::LEFT, 100); //swings to -100 deg, (locks left side)
    chassis.swingToHeading(-80, DriveSide::RIGHT, 100); //swings to -80 deg, (locks right side)
    chassis.swingToHeading(-90, DriveSide::LEFT, 100); //swings to -90 deg, (locks left side)
    chassis.moveToPose(-15, 33, -90, 500, {.forwards = true, .lead = 0, .maxSpeed = 100}, false);
    pros::delay(150);
    //matchload
    chassis.moveToPoint(15, 33, 1000, {.forwards = false}, false);
    setIntake(600);
    chassis.moveToPoint(23, 33, 2000, {.forwards = false, .maxSpeed = 60}, true);
    setIntake(-600);
    pros::delay(100);
    setIntake(600);
    pros::delay(50);
    setIntake(-600);
    pros::delay(50);
    setIntake(600);
    pros::delay(1450);
    pros::delay(300);
    chassis.moveToPose(6, 33, 135, 1300, {.forwards = true, .lead = 0, .maxSpeed = 90}, false); //move out after scoring
    setLittleWil(false);
    //long goal score
    holdIntake();
    chassis.moveToPose(30.5, 7, 135, 1300, {.forwards = true, .lead = .5}, false);
    chassis.turnToHeading(315, 800);
    chassis.moveToPoint(43.5, -9.5, 1700, {.forwards = false, .maxSpeed = 40}, false);
    setIntake(0); //temp
    setIntake(-600);
    pros::delay(100);
    scoreMiddle();
    pros::delay(100);
    setIntake(-600);
    pros::delay(100);
    scoreMiddle();
    pros::delay(1000);
    //middlescore
    chassis.moveToPoint(7, 21, 1700, {.forwards = true}, false);
    chassis.moveToPose(19, 21, -90, 500, {.forwards = false, .lead = 0}, false); //move out after scoring
    chassis.moveToPose(43, 21, -90, 1500, {.forwards = false, .lead = 0, .maxSpeed = 80}, false); //move out after scoring


   
}




void auton_red_right() {
   chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 32.5, 0, 1200, {.forwards = true, .lead = 0}, false); //initial forward movement
    chassis.turnToHeading(90, 700);
    setLittleWil(true);
    holdIntake();
    chassis.moveToPose(8, 37, 90, 500, {.forwards = true, .lead = 0}, false);
    chassis.swingToHeading(100, DriveSide::RIGHT, 100); //swings to -100 deg, (locks left side)
    chassis.swingToHeading(80, DriveSide::LEFT, 100); //swings to -80 deg, (locks right side)
    chassis.swingToHeading(90, DriveSide::RIGHT, 100); //swings to -90 deg, (locks left side)
    chassis.moveToPose(15, 37, 90, 500, {.forwards = true, .lead = 0, .maxSpeed = 100}, false);
    pros::delay(300);
    //matchload
    chassis.moveToPoint(-15, 37, 1000, {.forwards = false}, false);
    setIntake(600);
    chassis.moveToPoint(-23, 37, 2000, {.forwards = false, .maxSpeed = 60}, true);
    setIntake(-600);
    pros::delay(100);
    setIntake(600);
    pros::delay(50);
    setIntake(-600);
    pros::delay(50);
    setIntake(600);
    pros::delay(1450);
    pros::delay(600);
    setIntake(0);
    chassis.moveToPose(-6, 37, -135, 1300, {.forwards = true, .lead = 0, .maxSpeed = 90}, false); //move out after scoring
    setLittleWil(false);
    chassis.moveToPoint(-25, 10, 1300, {.forwards = true}, false); //move out after scoring
    chassis.moveToPose(-36.5, -6, -135, 1500, {.forwards = true, .lead = 0 , .maxSpeed = 60}, false); //move out after scoring
    setIntake(-600);
    pros::delay(100);
    setIntake(600);
    pros::delay(100);
    setIntake(-600);
    pros::delay(1300);
    chassis.moveToPoint(-2, 23, 2200, {.forwards = false, .maxSpeed = 90}, false); //move out after scoring
    chassis.moveToPose(-34, 20.5, -90, 2000, {.forwards = true, .lead = 0}, false);



    //long goal score


    // holdIntake();
    // chassis.moveToPose(30.5, 7, 135, 1300, {.forwards = true, .lead = .5}, false);
    // chassis.turnToHeading(315, 800);
    // chassis.moveToPoint(43.5, -9.5, 1700, {.forwards = false, .maxSpeed = 40}, false);
    // setIntake(0); //temp
    // setIntake(-600);
    // pros::delay(100);
    // scoreMiddle();
    // pros::delay(100);
    // setIntake(-600);
    // pros::delay(100);
    // scoreMiddle();
    // pros::delay(1000);
    // //middlescore
    // chassis.moveToPoint(7, 21, 1700, {.forwards = true}, false);
    // chassis.moveToPose(19, 21, -90, 500, {.forwards = false, .lead = 0}, false); //move out after scoring
    // chassis.moveToPose(43, 21, -90, 1500, {.forwards = false, .lead = 0, .maxSpeed = 80}, false); //move out after scoring




}




void auton_blue_left() {
    chassis.setPose(0, 0, 0);
}




void auton_blue_right() {
    chassis.setPose(0, 0, 0);
    holdIntake();
    chassis.moveToPose(-5, 20, 0, 1000, {.forwards = true, .lead = .4, .maxSpeed = 60}, false);
    chassis.moveToPose(11, 39, 0, 2200, {.forwards = true, .lead = 0, .maxSpeed = 60}, false);
    setLittleWil(true);
    pros::delay(300);
    setLittleWil(false);
    setIntake(0);
    chassis.moveToPose(35.5, 5, -180, 2200, {.forwards = false, .lead = 0, .maxSpeed = 100}, false);
    setLittleWil(true);
    chassis.moveToPoint(35.5, 30, 1500, {.forwards = false}, false);
    setIntake(600);
    pros::delay(3000);
    setIntake(0);
    chassis.moveToPoint(35.5, 10, 1500, {.forwards = true}, false);
    setWing(true);
    chassis.moveToPose(22, 42, 0, 2200, {.forwards = true, .maxSpeed = 80}, false);
}




void autonomous() {
    // auton_swap();
    // auton_red_left();
    auton_red_right();
    //auton_blue_left();
    //auton_blue_right();
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
    while (true) {
        // Get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        chassis.tank(leftY, rightY);


        // Intake control logic
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_velocity(600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            hoodPiston.set_value(false); // Set the hood piston to close
            intake.move_velocity(600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move_velocity(-600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            hoodPiston.set_value(false); // Set the hood piston to close
            firstStage.move_velocity(600);
            hood.move_velocity(-200);
        } else {
            intake.move_velocity(0);
            hoodPiston.set_value(true); // Set the hood piston to open
        }




        // Little Wil toggle logic
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            littleWil_state = !littleWil_state;
            littleWil.set_value(littleWil_state); // Set the Little Wil piston to the new state
        }




        // Wing toggle logic
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            Wing_state = !Wing_state;
            Wing.set_value(Wing_state); // Set the Wing piston to the new state
        }




        // Delay to save resources
        pros::delay(25);
    }
}
