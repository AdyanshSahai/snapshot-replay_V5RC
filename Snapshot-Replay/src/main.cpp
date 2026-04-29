#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cstdio>
#include <cstring>

/*
Ports:
Left Drivetrain: 10, 3, 4
Right Drive: 5, 7, 8
Bottom rollers: 13
Middle roller: -14
Top roller: -16
Scraper: A (Port 1)
Wing: B (Port 2)
Radio: 21
Optical: 20
Y axis encoder: -17
Gyro: 19
*/

// button edge states for pressed()
bool prevUp    = false;
bool prevDown  = false;
bool prevLeft  = false;
bool prevRight = false;
bool prevB     = false;
bool prevY     = false;
bool prevX     = false;

// pose snapshot, updated each loop tick
double posX;
double posY;
double posTheta;

// roller velocity snapshots for recording
int bottomSpeed = 0;
int topSpeed    = 0;


// ---- HARDWARE ----

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor bottomRoller(-3);
pros::Motor topRoller(-2);

pros::MotorGroup leftMotors({10, 9, 8}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-7, -6, -5}, pros::MotorGearset::blue);

pros::Imu     imu(19);
pros::Optical optical_sensor(20);

pros::adi::DigitalOut scraper('B', false);
pros::adi::DigitalOut wing('A', false);

bool wingState    = false;
bool scraperState = false; 

lemlib::TrackingWheel leftIME(&leftMotors,   lemlib::Omniwheel::NEW_325, -5.4, 450);
lemlib::TrackingWheel rightIME(&rightMotors, lemlib::Omniwheel::NEW_325,  5.4, 450);


// ---- LEMLIB SETUP ----

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              10.8,
                              lemlib::Omniwheel::NEW_325,
                              450,
                              6
);

lemlib::ControllerSettings linearController(15, 0, 2, 0, 0, 0, 0, 0, 0);

lemlib::ControllerSettings angularController(2, 0, 10, 0, 0.5, 100, 2, 500, 0);

lemlib::OdomSensors sensors(&leftIME, &rightIME, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(5, 10, 1.04);
lemlib::ExpoDriveCurve steerCurve(5, 10, 1.04);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


// returns true only on the rising edge of a button press
bool pressed(bool current, bool &previous) {
    if (current && !previous) {
        previous = current;
        return true;
    }
    previous = current;
    return false;
}


// ---- REPLAY EXECUTION ----
// takes one line from the SD card file and runs the matching action.
// token format is whatever opcontrol writes, e.g. "MOVE_POSE_FWD 12.00 -5.00 90.00"
// newline is stripped before any comparisons so fgets doesn't mess up strcmp.
void execute_command(char* line) {
    line[strcspn(line, "\n")] = 0;  // strip newline
    if (line[0] == '\0') return;    // skip blank

    if (strncmp(line, "MOVE_POSE_FWD", 13) == 0) {        // forward move
        double x, y, theta;
        if (sscanf(line, "MOVE_POSE_FWD %lf %lf %lf", &x, &y, &theta) == 3) {
            chassis.moveToPose(x, y, theta, 2000, {.forwards = true});
            chassis.waitUntilDone();
        }
    } else if (strncmp(line, "MOVE_POSE_BWD", 13) == 0) { // backward move
        double x, y, theta;
        if (sscanf(line, "MOVE_POSE_BWD %lf %lf %lf", &x, &y, &theta) == 3) {
            chassis.moveToPose(x, y, theta, 2000, {.forwards = false});
            chassis.waitUntilDone();
        }
    } else if (strncmp(line, "TURN_TO", 7) == 0) {        // heading turn
        double theta;
        if (sscanf(line, "TURN_TO %lf", &theta) == 1) {
            chassis.turnToHeading(theta, 1000);
            chassis.waitUntilDone();
        }
    } else if (strncmp(line, "INTAKE_STATE", 12) == 0) {  // roller state
        int l1, l2, r1, r2;
        if (sscanf(line, "INTAKE_STATE %d %d %d %d", &l1, &l2, &r1, &r2) == 4) {
            if (l2) {
                bottomRoller.move(-127); topRoller.brake();
            } else if (l1) {
                bottomRoller.move(127);  topRoller.move(127);
            } else if (r2) {
                bottomRoller.move(-127); topRoller.brake();
            } else if (r1) {
                bottomRoller.move(-127); topRoller.move(-127);
            } else {
                bottomRoller.brake(); topRoller.brake();
            }
        }
    } else if (strcmp(line, "SCRAPER_TOGGLE") == 0) {      // flip scraper
        scraperState = !scraperState;
        scraper.set_value(scraperState);
        pros::delay(200);   // piston settle
    } else if (strcmp(line, "WING_TOGGLE") == 0) {        // flip wing
        wingState = !wingState;
        wing.set_value(wingState);
        pros::delay(200);   // piston settle
    }
}

void initialize() {
    pros::lcd::initialize();

    topRoller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    optical_sensor.set_led_pwm(100);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            pros::lcd::print(4, "Brain Battery %.0f%%",      pros::battery::get_capacity());
            pros::lcd::print(5, "Controller Battery: %.0f%%", controller.get_battery_capacity());
            //pros::lcd::print(6, "Hue: %f", optical.get_hue());

            // pros::delay to save resources
            pros::delay(50);
        }
    });
}



// ---- DISABLED / COMPETITION INIT ----
void disabled() {}
void competition_initialize() {}


void autonomous() {
    // ---- REPLAY ----
    // open the token file that was built during driver control and run each
    // line through execute_command. if the file isn't there, nothing happens.
    FILE* f = fopen("/usd/replay.txt", "r"); // open replay
    if (f) {
        pros::lcd::print(0, "Replaying...");    // show status
        char line[256];                         // line buffer
        while (fgets(line, sizeof(line), f) != NULL) {
            execute_command(line);              // run command
        }
        fclose(f);                              // close file
        pros::delay(1000);                      // brief settle
        pros::lcd::print(0, "Replay Done");     // done message
    }
}


void opcontrol() {

    // ---- ARCHIVE OLD REPLAY ----
    // back up whatever was recorded last run before we start writing new stuff.
    // only runs at the top of driver control, so autonomous replay is untouched.
    FILE* src = fopen("/usd/replay.txt", "r");           // open old replay
    FILE* dst = fopen("/usd/replay-archive.txt", "w");   // overwrite archive
    if (src && dst) {
        char buf[256];                                    // copy buffer
        while (fgets(buf, sizeof(buf), src) != NULL) {
            fputs(buf, dst);                              // write line
        }
    }
    if (src) fclose(src);                                // close source
    if (dst) fclose(dst);                                // close archive

    // Open replay file ONCE for the entire match.
    // Reopening every button press exhausts VexOS's SD file-handle
    // budget (~8 total opens per run) after only a few presses.
    FILE* replay = fopen("/usd/replay.txt", "w");        // truncate + keep open
    if (!replay) { pros::lcd::print(6, "USD OPEN FAILED"); }

    while (true) {

        // ---- DRIVE ----
        // left stick is throttle, right stick is turn, curvature mode
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   // throttle value
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // steering value
        chassis.curvature(leftY, -rightX);                                       // move robot


        // ---- BUTTON READS ----
        // snapshot everything once per tick so each button is checked consistently
        bool L1    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);    // intake full
        bool L2    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);    // hold outtake
        bool R1    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);    // top out
        bool R2    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);    // bottom out
        bool up    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);    // save fwd
        bool down  = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);  // save bwd
        bool right = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT); // save intake
        bool left  = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);  // save turn
        bool b     = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);     // scraper toggle
        bool y     = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);     // wing toggle
        bool x     = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);     // close + stop


        // ---- INTAKE / OUTTAKE ----
        // L2 dumps bottom only (top holds), L1 runs both in,
        // R2 same as L2, R1 runs both out
        if (L2) {
            bottomRoller.move(-127);  // outtake bottom
            topRoller.brake();        // hold top
        } else if (L1) {
            bottomRoller.move(-70);   // intake bottom
            topRoller.move(90);      // intake top
        } else if (R2) {
            bottomRoller.move(127);   // intake bottom
            topRoller.move(127);      // intake top
        } else if (R1) {
            bottomRoller.move(-127);  // outtake bottom
            topRoller.move(-127);     // outtake top
        } else {
            bottomRoller.brake();     // stop bottom
            topRoller.brake();        // stop top
        }

        
        // ---- SCRAPER ----
        // B toggles the scraper each press, same pattern as wing on Y
        if (pressed(b, prevB)) {
            scraperState = !scraperState;
            scraper.set_value(scraperState);    // toggle scraper
            if (replay) {
                fprintf(replay, "SCRAPER_TOGGLE\n");
                fflush(replay);
                pros::lcd::print(6, "Saved Scraper");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }
        }


        // ---- WING ----
        // Y flips the wing each press. toggle and record both happen here so
        // pressed() is only called once — calling it twice would miss the record.
        if (pressed(y, prevY)) {
            wingState = !wingState;
            wing.set_value(wingState);          // toggle wing
            if (replay) {
                fprintf(replay, "WING_TOGGLE\n");
                fflush(replay);
                pros::lcd::print(6, "Saved Wing");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }
        }


        // ---- POSE SNAPSHOT ----
        // grab pose once so all the recording blocks below see the same values
        lemlib::Pose pose = chassis.getPose(); // read pose
        posX     = pose.x;      // store x
        posY     = pose.y;      // store y
        posTheta = pose.theta;  // store heading

        bottomSpeed = bottomRoller.get_actual_velocity(); // bottom rpm
        topSpeed    = topRoller.get_actual_velocity();    // top rpm


        // ---- RECORDING ----
        // d-pad writes tokenized commands to the SD card for replay in auto.
        // UP = forward moveToPose, DOWN = backward, LEFT = turn, RIGHT = intake state.
        // scraper on/off go in a separate chain so they don't block the d-pad.
        if (pressed(up, prevUp)) {
            if (replay) {
                fprintf(replay, "MOVE_POSE_FWD %.2f %.2f %.2f\n", posX, posY, posTheta);
                fflush(replay);
                pros::lcd::print(6, "Saved FWD");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }

        } else if (pressed(down, prevDown)) {
            if (replay) {
                fprintf(replay, "MOVE_POSE_BWD %.2f %.2f %.2f\n", posX, posY, posTheta);
                fflush(replay);
                pros::lcd::print(6, "Saved BWD");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }

        } else if (pressed(left, prevLeft)) {
            if (replay) {
                fprintf(replay, "TURN_TO %.2f\n", posTheta);
                fflush(replay);
                pros::lcd::print(6, "Saved Turn");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }

        } else if (pressed(right, prevRight)) {
            if (replay) {
                fprintf(replay, "INTAKE_STATE %d %d %d %d\n", (int)L1, (int)L2, (int)R1, (int)R2);
                fflush(replay);
                pros::lcd::print(6, "Saved Intake");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }
        }


        // ---- STOP RECORDING ----
        // X closes the file and exits driver control loop
        if (pressed(x, prevX)) {
            if (replay) { fclose(replay); replay = nullptr; }
            pros::lcd::print(6, "Recording Closed");
            break;
        }

        pros::delay(20);  // loop rate
    }
}
