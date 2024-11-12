package org.firstinspires.ftc.teamcode.Utilities;

public class Constants {

    public enum OpModes {
        TELE_OP,
        AUTO,
        PRE_MATCH_INIT
    }

    public static final class Drive {

        //Speed/Acceleration Limits
        public static final double MAX_LINEAR_SPEED = 2.0; //Inches/Second
        public static final double MAX_ANGULAR_SPEED = 1.0; //Radians/Second
        public static final double MAX_LINEAR_ACCELERATION = 8.0; //Inches/Second^2
        public static final double MAX_ANGULAR_ACCELERATION = 6.0; //Radians/Second^2

        public static final int MOTOR_MAX_VELOCITY = 1800; //Ticks/Second

        // Constants for motor names used in the configuration
        public static final String LEFT_FRONT_DRIVE_MOTOR_NAME = "lfMotor";    // Name for the left front motor
        public static final String RIGHT_FRONT_DRIVE_MOTOR_NAME = "rfMotor";   // Name for the right front motor
        public static final String LEFT_BACK_DRIVE_MOTOR_NAME = "lbMotor";     // Name for the left back motor
        public static final String RIGHT_BACK_DRIVE_MOTOR_NAME = "rbMotor";    // Name for the right back motor

        //PID Coefficients for the linear drive 2D controller
        public static final double LINEAR_KP = 0.001;
        public static final double LINEAR_KD = 0.0001;
        public static final double LINEAR_KI = 0.00001;

        //PID Coefficients for the angular drive controller
        public static final double ANGULAR_KP = 0.001;
        public static final double ANGULAR_KD = 0.0001;
        public static final double ANGULAR_KI = 0.00001;

        //How much turning will override drive speed
        public static final double TURN_OVERRIDE = 0.25;

    }

    public enum ArmPosition {
        IDLE(
            0,      // Shoulder position
            0,      // Linear slide position
            0.0     // Wrist servo position
        ),

        SAMPLE_INTAKE(
            440,      // Shoulder position
            2672,      // Linear slide position
            0.0     // Wrist servo position
        ),

        SPECIMEN_INTAKE(
            41,      // Shoulder position
            0,      // Linear slide position
            0.0     // Wrist servo position
        ),

        BASKET_ONE(
            561,      // Shoulder position
            0,      // Linear slide position
            0.0     // Wrist servo position
        ),

        BASKET_TWO(
            607,      // Shoulder position
            2590,      // Linear slide position
            0.0     // Wrist servo position
        ),

        CHAMBER_ONE(
            0,      // Shoulder position
            0,      // Linear slide position
            0.0     // Wrist servo position
        ),

        CHAMBER_TWO(
            378,      // Shoulder position
            0,      // Linear slide position
            0.0     // Wrist servo position
        ),

        CLIMB(
            400,      // Shoulder position
            0,      // Linear slide position
            0.0     // Wrist servo position
        );


        public int SHOULDER_POSITION;
        public int LINEAR_SLIDE_POSITION;
        public double WRIST_POSITION;

        //a constructor class to save the positions to each location
        ArmPosition(int shoulderPosition, int linearSlidePosition, double wristPosition) {
            SHOULDER_POSITION = shoulderPosition;
            LINEAR_SLIDE_POSITION = linearSlidePosition;
            WRIST_POSITION = wristPosition;
        }
    }

    public static final class Arm {

        // Constants for motor names used in the configuration
        public static final String SHOULDER_MOTOR_NAME = "shoulderMotor";    // Name for the shoulder motor
        public static final String LINEAR_SLIDE_MOTOR_NAME = "linearSlideMotor";   // Name for the linear slide motor

        //PID Coefficients for the shoulder
        public static final double SHOULDER_KP = 0.002;
        public static final double SHOULDER_KI = 0.0;
        public static final double SHOULDER_KD = 0.0;

        //PID Coefficients for the linear slide
        public static final double LINEAR_SLIDE_KP = 0.01;
        public static final double LINEAR_SLIDE_KI = 0.0;
        public static final double LINEAR_SLIDE_KD = 0.0;

        public static double getArmFValue(int shoulderPosition, int linearSlidePosition) {

            //finding the angle of the arm
            double angle = 0.002176*shoulderPosition-0.6853;

            //adjusting the slide position due to the angle
            double adjustedLinearSlidePosition = linearSlidePosition * Math.cos(angle);

            if (adjustedLinearSlidePosition < 1000) {
                return 0;
            } else  {
                return 0.00001*adjustedLinearSlidePosition-0.01;
            }

        }

    }

    public static final class Manipulator {
        // Constants for servo names used in the configuration
        public static final String WRIST_SERVO_NAME = "wristServo";    // Name for the wrist servo
        public static final String CLAW_SERVO_NAME = "clawServo";   // Name for the claw servo
        public static final String ROLLER_SERVO_NAME = "rollerServo"; // Name for the roller servo

        //Claw Servo Positions
        public static final double CLAW_OPEN_POSITION = 0.4;
        public static final double CLAW_CLOSED_POSITION = 0.15;

        //Roller Servo Speeds
        public static final double ROLLER_INTAKE_SPEED = 1.0;
        public static final double ROLLER_OUTTAKE_SPEED = -1.0;
        public static final double ROLLER_STOPPED_SPEED = 0.0;


    }

    public static final class Core {

        public static final String IMU_NAME = "imu";
        public static final boolean IMU_REVERSED = true;

    }

}
