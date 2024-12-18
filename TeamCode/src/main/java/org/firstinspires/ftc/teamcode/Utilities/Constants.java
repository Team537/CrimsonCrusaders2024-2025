package org.firstinspires.ftc.teamcode.Utilities;

import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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

        public static final double DISPLACEMENT_PER_TICKS = 0.00554; //Inches/Ticks

        //Values to curve the input to essentially make it easier to drive slower
        public static final double LINEAR_INPUT_CURVE_POWER = 2.5;
        public static final double ANGULAR_INPUT_CURVE_POWER = 2.5;

        // Constants for motor names used in the configuration
        public static final String LEFT_FRONT_DRIVE_MOTOR_NAME = "lfMotor";    // Name for the left front motor
        public static final String RIGHT_FRONT_DRIVE_MOTOR_NAME = "rfMotor";   // Name for the right front motor
        public static final String LEFT_BACK_DRIVE_MOTOR_NAME = "lbMotor";     // Name for the left back motor
        public static final String RIGHT_BACK_DRIVE_MOTOR_NAME = "rbMotor";    // Name for the right back motor

        //PID Coefficients for the linear drive 2D controller
        public static final double LINEAR_KP = 0.04;
        public static final double LINEAR_KI = 0.00000;
        public static final double LINEAR_KD = 0.0004;

        //PID Coefficients for the angular drive controller
        public static final double ANGULAR_KP = 0.4;
        public static final double ANGULAR_KI = 0.00000;
        public static final double ANGULAR_KD = 0.02;

        //How much turning will override drive speed
        public static final double TURN_OVERRIDE = 0.25;

        public static final double MINIMUM_TARGET_LINEAR_ERROR = 1.0; //Inches
        public static final double MINIMUM_TARGET_ANGULAR_ERROR = 0.1; //Radians

    }

    public enum ArmPosition {
        IDLE(
            0,      // Shoulder position
            0      // Linear slide position
        ),

        SAMPLE_INTAKE(
            270,      // Shoulder position
            2672      // Linear slide position
        ),

        SPECIMEN_INTAKE(
            150,      // Shoulder position
            0      // Linear slide position
        ),

        BASKET_ONE(
            680,      // Shoulder position
            1300      // Linear slide position
        ),

        BASKET_TWO(
            870,      // Shoulder position
            3000      // Linear slide position
        ),

        CHAMBER_ONE(
            160,      // Shoulder position
            0      // Linear slide position
        ),

        CHAMBER_TWO(
            500,      // Shoulder position
            0      // Linear slide position
        ),

        CLIMB(
            560,      // Shoulder position
            0      // Linear slide position
        );


        public int SHOULDER_POSITION;
        public int LINEAR_SLIDE_POSITION;

        //a constructor class to save the positions to each location
        ArmPosition(int shoulderPosition, int linearSlidePosition) {
            SHOULDER_POSITION = shoulderPosition;
            LINEAR_SLIDE_POSITION = linearSlidePosition;
        }
    }

    public static final class Arm {

        // Constants for motor names used in the configuration
        public static final String SHOULDER_MOTOR_NAME = "shoulderMotor";    // Name for the shoulder motor
        public static final String LINEAR_SLIDE_MOTOR_NAME = "linearSlideMotor";   // Name for the linear slide motor

        //PID Coefficients for the shoulder
        public static final double SHOULDER_KP = 0.002;
        public static final double SHOULDER_KI = 0.0;
        public static final double SHOULDER_KD = 0.0002;

        //PID Coefficients for the linear slide
        public static final double LINEAR_SLIDE_KP = 0.001;
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

        public static final int SHOULDER_MINIMUM_ERROR = 40;
        public static final int LINEAR_SLIDE_MINIMUM_ERROR = 200;

    }

    public static final class Manipulator {
        // Constants for servo names used in the configuration
        public static final String WRIST_SERVO_NAME = "wristServo";    // Name for the wrist servo
        public static final String CLAW_SERVO_NAME = "clawServo";   // Name for the claw servo
        public static final String ROLLER_SERVO_NAME = "rollerServo"; // Name for the roller servo

        //Claw Servo Positions
        public static final double CLAW_OPEN_POSITION = 0.3;
        public static final double CLAW_CLOSED_POSITION = 0.05;

        //Roller Servo Speeds
        public static final double ROLLER_INTAKE_SPEED = 1.0;
        public static final double ROLLER_OUTTAKE_SPEED = -0.2;
        public static final double ROLLER_STOPPED_SPEED = 0.0;
        public static final double ROLLER_HOLD_SPEED = 0.1;

        //Set positions for the wrist
        public static final double IDLE_WRIST_POSITION = 1.0;
        public static final double SAMPLE_INTAKE_WRIST_POSITION = 0.0;
        public static final double GRAB_SPECIMEN_INTAKE_WRIST_POSITION = 0.65;
        public static final double RETRACT_SPECIMEN_INTAKE_WRIST_POSITION = 1.0;

        public static final double BASKET_ONE_WRIST_POSITION = 0.0;
        public static final double BASKET_TWO_WRIST_POSITION = 0.0;

        public static final double HOVER_CHAMBER_ONE_WRIST_POSITION = 1.0;
        public static final double PLACE_CHAMBER_ONE_WRIST_POSITION = 0.65;
        public static final double HOVER_CHAMBER_TWO_WRIST_POSITION = 0.7;
        public static final double PLACE_CHAMBER_TWO_WRIST_POSITION = 0.35;

        public static final double CLIMB_WRIST_POSITION = 1.0;

        public static final double ROLLER_OUTTAKE_MINIMUM_TIME = 0.8;

    }

    public static final class Vision {

        public static final class Localization {

            public static final String LOCALIZATION_CAMERA_NAME = "LocalizationCamera";

            //Coordinate Frame of the Camera (reference to the center of the robot, on the floor and facing towards the manipulator)
            public static final Position LOCALIZATION_CAMERA_POSITION = new Position(DistanceUnit.INCH, -4,  -7, 7.375,0);
            public static final YawPitchRollAngles LOCALIZATION_CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.RADIANS, Math.PI, -0.5 * Math.PI, Math.PI, 0);

        }

    }

    public static final class Core {

        public static final String IMU_NAME = "imu";
        public static final boolean IMU_REVERSED = false;

    }

    public static final class Arena {

        /**
         * inverts the pose to simplify time copying poses over to the blue alliance
         * @param pose the pose sent (a red alliance pose)
         * @return the inverted pose (for blue alliance)
         */
        private static Pose2D invertPose(Pose2D pose) {
            return new Pose2D(
                DistanceUnit.INCH,
                -pose.getX(DistanceUnit.INCH),
                -pose.getY(DistanceUnit.INCH),
                AngleUnit.RADIANS,
                Math.PI + pose.getHeading(AngleUnit.RADIANS)
            );
        };

        public static final Rotation2d RED_ALLIANCE_OFFSET = new Rotation2d(0);
        public static final Rotation2d BLUE_ALLIANCE_OFFSET = new Rotation2d(Math.PI);


        //location of important field locations for auto
        public static final Pose2D RED_SAMPLE_SCORE_SEEK_POSE = new Pose2D(DistanceUnit.INCH,-48,-48,AngleUnit.RADIANS,0);
        public static final Pose2D BLUE_SAMPLE_SCORE_SEEK_POSE = invertPose(RED_SAMPLE_SCORE_SEEK_POSE);
        public static final Pose2D RED_SAMPLE_SCORE_POSE = new Pose2D(DistanceUnit.INCH,-48,-48,AngleUnit.RADIANS,-0.75 * Math.PI);
        public static final Pose2D BLUE_SAMPLE_SCORE_POSE = invertPose(RED_SAMPLE_SCORE_POSE);

        //The positions for lining up and intaking samples
        public static final Pose2D RED_SAMPLE_ONE_LINE_UP_POSE = new Pose2D(DistanceUnit.INCH, -21, -45, AngleUnit.RADIANS, 2.458);
        public static final Pose2D BLUE_SAMPLE_ONE_LINE_UP_POSE = invertPose(RED_SAMPLE_ONE_LINE_UP_POSE);

        public static final Pose2D RED_SAMPLE_ONE_INTAKE_POSE = new Pose2D(DistanceUnit.INCH, -23, -44, AngleUnit.RADIANS, 2.458);
        public static final Pose2D BLUE_SAMPLE_ONE_INTAKE_POSE = invertPose(RED_SAMPLE_ONE_INTAKE_POSE);

        // Sample Two Poses (x decreased by 10 inches)
        public static final Pose2D RED_SAMPLE_TWO_LINE_UP_POSE = new Pose2D(DistanceUnit.INCH, -31, -45, AngleUnit.RADIANS, 2.458);
        public static final Pose2D BLUE_SAMPLE_TWO_LINE_UP_POSE = invertPose(RED_SAMPLE_TWO_LINE_UP_POSE);

        public static final Pose2D RED_SAMPLE_TWO_INTAKE_POSE = new Pose2D(DistanceUnit.INCH, -33, -44, AngleUnit.RADIANS, 2.458);
        public static final Pose2D BLUE_SAMPLE_TWO_INTAKE_POSE = invertPose(RED_SAMPLE_TWO_INTAKE_POSE);

        // Sample Three Poses (x decreased by another 10 inches)
        public static final Pose2D RED_SAMPLE_THREE_LINE_UP_POSE = new Pose2D(DistanceUnit.INCH, -41, -45, AngleUnit.RADIANS, 2.458);
        public static final Pose2D BLUE_SAMPLE_THREE_LINE_UP_POSE = invertPose(RED_SAMPLE_THREE_LINE_UP_POSE);

        public static final Pose2D RED_SAMPLE_THREE_INTAKE_POSE = new Pose2D(DistanceUnit.INCH, -43, -44, AngleUnit.RADIANS, 2.458);
        public static final Pose2D BLUE_SAMPLE_THREE_INTAKE_POSE = invertPose(RED_SAMPLE_THREE_INTAKE_POSE);

        public static final Pose2D RED_READY_CLIMB_POSE = new Pose2D(DistanceUnit.INCH, -44, 0, AngleUnit.RADIANS, 1.11);
        public static final Pose2D BLUE_READY_CLIMB_POSE = invertPose(RED_READY_CLIMB_POSE);

        public static final Pose2D RED_CLIMB_POSE = new Pose2D(DistanceUnit.INCH, -20, 0, AngleUnit.RADIANS, 0);
        public static final Pose2D BLUE_CLIMB_POSE = invertPose(RED_CLIMB_POSE);

    }

}
