package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.RobotVision.VisionLocalizationSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.RateLimiter;
import org.firstinspires.ftc.teamcode.Utilities.RateLimiter2D;

public class DriveSubsystem extends Subsystem {

    //saving motors
    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private IMU imu;

    private Rotation2d imuOffset = new Rotation2d(0);

    private RateLimiter2D linearRateLimiter;
    private RateLimiter angularRateLimiter;

    //getting the PIDs
    private PID angularPID;
    private PID linearXPID;
    private PID linearYPID;

    //Variables for saving odometry position
    private Vector2d robotPosition = new Vector2d();

    private int lastLeftFrontPosition = 0;
    private int lastRightFrontPosition = 0;
    private int lastLeftBackPosition = 0;
    private int lastRightBackPosition = 0;

    private Pose2D targetPose =  new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0);

    boolean haveCameraSetOrientation = false;
    /**
     * Constructor class for the drive subsystem; can only be accessed by the builder
     * @param leftFrontMotor object for left front motor
     * @param rightFrontMotor object for right front motor
     * @param leftBackMotor object for left back motor
     * @param rightBackMotor object for right back motor
     */
    private DriveSubsystem(
        DcMotorEx leftFrontMotor,
        DcMotorEx rightFrontMotor,
        DcMotorEx leftBackMotor,
        DcMotorEx rightBackMotor,
        IMU imu
    ) {
        this.leftFrontMotor = leftFrontMotor;     // Assign the left front motor to the instance
        this.rightFrontMotor = rightFrontMotor;   // Assign the right front motor to the instance
        this.leftBackMotor = leftBackMotor;       // Assign the left back motor to the instance
        this.rightBackMotor = rightBackMotor;     // Assign the right back motor to the instance
        this.imu = imu;

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        angularPID = new PID(
            Constants.Drive.ANGULAR_KP,
            Constants.Drive.ANGULAR_KI,
            Constants.Drive.ANGULAR_KD
        );

        linearXPID = new PID(
            Constants.Drive.LINEAR_KP,
            Constants.Drive.LINEAR_KI,
            Constants.Drive.LINEAR_KD
        );

       linearYPID = new PID(
            Constants.Drive.LINEAR_KP,
            Constants.Drive.LINEAR_KI,
            Constants.Drive.LINEAR_KD
        );

        angularRateLimiter = new RateLimiter.Builder()
            .setLowerBound(-1)
            .setUpperBound(1)
            .setMaxRate(Constants.Drive.MAX_ANGULAR_ACCELERATION / Constants.Drive.MAX_ANGULAR_SPEED)
            .setInitialValue(0)
            .build();

        linearRateLimiter = new RateLimiter2D(Constants.Drive.MAX_LINEAR_ACCELERATION / Constants.Drive.MAX_LINEAR_SPEED,new Vector2d());
    }

    /**
     * Drives the robot using gamepad inputs
     * @param gamepad the gamepad object that it would read
     */
    public void driveFromGamepad(Gamepad gamepad) {
        Vector2d linearSpeed = new Vector2d(
            gamepad.left_stick_x,
            gamepad.left_stick_y
        );
        linearSpeed = linearSpeed.scale(Math.pow(linearSpeed.magnitude(),Constants.Drive.LINEAR_INPUT_CURVE_POWER));

        //rotating the input if the driver is facing a different direction
        if (Robot.getInstance().robotContainer.alliance == Alliance.RED) {
            linearSpeed.rotateBy(Constants.Arena.RED_ALLIANCE_OFFSET.getDegrees());
        } else {
            linearSpeed.rotateBy(Constants.Arena.BLUE_ALLIANCE_OFFSET.getDegrees());
        }

        double angularSpeed = Math.pow(Math.abs(gamepad.right_stick_x),Constants.Drive.ANGULAR_INPUT_CURVE_POWER) * Math.signum(gamepad.right_stick_x);
        if (gamepad.guide) {
            if (Robot.getInstance().robotContainer.alliance == Alliance.RED) {
                setIMU(new Rotation2d(Math.PI/2));
            } else {
                setIMU(new Rotation2d(-Math.PI/2));
            }
        }
        drive(linearSpeed.times(Constants.Drive.MAX_LINEAR_SPEED),angularSpeed*Constants.Drive.MAX_ANGULAR_SPEED);
    }

    public void setIMU(Rotation2d rotation) {
        imuOffset = rotation.minus(getHeading()).plus(imuOffset);
    }

    public void testWheelsFromGamepad(Gamepad gamepad) {
        // Control left front motor with gamepad 'a'
        if (gamepad.a) {
            leftFrontMotor.setVelocity(2000);
        } else {
            leftFrontMotor.setVelocity(0);
        }

        // Control right front motor with gamepad 'b'
        if (gamepad.b) {
            rightFrontMotor.setVelocity(2000);
        } else {
            rightFrontMotor.setVelocity(0);
        }

        // Control left back motor with gamepad 'x'
        if (gamepad.x) {
            leftBackMotor.setVelocity(2000);
        } else {
            leftBackMotor.setVelocity(0);
        }

        // Control right back motor with gamepad 'y'
        if (gamepad.y) {
            rightBackMotor.setVelocity(2000);
        } else {
            rightBackMotor.setVelocity(0);
        }

        Robot.getInstance().opMode.telemetry.addData("lfSpeed", leftFrontMotor.getVelocity());
        Robot.getInstance().opMode.telemetry.addData("rfSpeed", rightFrontMotor.getVelocity());
        Robot.getInstance().opMode.telemetry.addData("lbSpeed", leftBackMotor.getVelocity());
        Robot.getInstance().opMode.telemetry.addData("rbSpeed", rightBackMotor.getVelocity());

    }

    public void confirmLinearDirectionsFromGamepad(Gamepad gamepad) {
        setIMU(new Rotation2d());
        if (gamepad.a) {
            drive(new Vector2d(0,-1 * Constants.Drive.MAX_LINEAR_SPEED), 0);
        } else if (gamepad.b) {
            drive(new Vector2d(1 * Constants.Drive.MAX_LINEAR_SPEED,0), 0);
        } else if (gamepad.x) {
            drive(new Vector2d(-1 * Constants.Drive.MAX_LINEAR_SPEED,0), 0);
        } else if (gamepad.y) {
            drive(new Vector2d(0,1 * Constants.Drive.MAX_LINEAR_SPEED), 0);
        } else {
            drive(new Vector2d(0,0),0);
        }
    }

    /**
     * sets the target pose
     * @param pose the pose to set to
     */
    public void setTargetPosition(Pose2D pose) {
        targetPose = pose;
    }

    /**
     * drives to a position
     * @param maxLinearSpeed the maximum speed it will go to that location in inches per second
     * @param maxAngularSpeed the maximum rotational speed it will take to that location in radians per second
     */
    public void driveToPosition(double maxLinearSpeed, double maxAngularSpeed) {

        double unitMaxLinearSpeed = maxLinearSpeed / Constants.Drive.MAX_LINEAR_SPEED;
        double unitMaxAngularSpeed = maxAngularSpeed / Constants.Drive.MAX_ANGULAR_SPEED;

        //find the linear velocity using PID controllers
        Vector2d linearVelocity = new Vector2d(
            -linearXPID.calculate(robotPosition.getX() - targetPose.getX(DistanceUnit.INCH)),
            linearYPID.calculate(robotPosition.getY() - targetPose.getY(DistanceUnit.INCH))
        );

        //find the angular velocity using the PID controller
        double angularVelocity = angularPID.calculate(getHeading().minus(new Rotation2d(targetPose.getHeading(AngleUnit.RADIANS))).getRadians());

        //clamp the linear velocity by multiplying it by the hyperbolic tangent of its magnitude.
        //if the magnitude is zero, then just set the clamped vector to zero.
        Vector2d clampedLinearVelocity;
        if (linearVelocity.magnitude() < 1e-4) {
            clampedLinearVelocity = new Vector2d(0,0);
        } else {
            clampedLinearVelocity = linearVelocity.normalize().times(Math.tanh(linearVelocity.magnitude()));
        }

        //clamp the angular velocity by plugging it into the hyperbolic tangent function
        double clampedAngularVelocity = Math.tanh(angularVelocity);

        //If the linear velocity is larger than the max, clamp it
        if (clampedLinearVelocity.magnitude() > unitMaxLinearSpeed) {
            clampedLinearVelocity.normalize().times(unitMaxLinearSpeed);
        }

        //If the angular velocity is larger than the max, clamp it
        clampedAngularVelocity = Math.max(-unitMaxAngularSpeed, Math.min(unitMaxAngularSpeed, clampedAngularVelocity));

        //drive using the calculated PID values
        drive(clampedLinearVelocity.times(Constants.Drive.MAX_LINEAR_SPEED),clampedAngularVelocity * Constants.Drive.MAX_ANGULAR_SPEED);

    }

    /**
     * finds the distance to a target
     * @return the distance (in inches)
     */
    public double getDistanceToPose() {
        return new Vector2d(
            targetPose.getX(DistanceUnit.INCH) - robotPosition.getX(),
            targetPose.getY(DistanceUnit.INCH) - robotPosition.getY()
        ).magnitude();
    }

    /**
     * finds the angle to a target
     * @return the angle (in radians)
     */
    public double getAngleToPose() {
        return getHeading().minus(
            new Rotation2d(targetPose.getHeading(AngleUnit.RADIANS))
        ).getRadians();
    }

    //Saves the value for intaking the specimen during manual control, allows the robot to slide along to intake
    double SpecimenIntakePosition;

    /**
     * Manually drives in a good way to pick up specimen
     * @param gamepad The gamepad used
     */
    public void specimenIntakeFromGamepad(Gamepad gamepad) {

    }

    public void resetSpecimenIntakePosition() {
        
    }

    public void setHaveCameraSetOrientation(boolean state) {
        haveCameraSetOrientation = state;
    }

    /**
     * Drives using the desried speeds
     * @param targetLinearSpeed the linear speed, or the speed in a direction
     * @param targetAngularSpeed the rotational speed, or how fast it will rotate
     */
    public void drive(Vector2d targetLinearSpeed, double targetAngularSpeed) {

        Robot.getInstance().opMode.telemetry.addData("Linear Speed",targetLinearSpeed);
        Robot.getInstance().opMode.telemetry.addData("Angular Speed",targetAngularSpeed);

        Vector2d unitLinearSpeed = targetLinearSpeed.div(Constants.Drive.MAX_LINEAR_SPEED);
        double unitAngularSpeed = targetAngularSpeed / Constants.Drive.MAX_ANGULAR_SPEED;

        //clamping the target speed if a too large speed was provided
        if (unitLinearSpeed.magnitude() > 1) {
            unitLinearSpeed = unitLinearSpeed.div(unitLinearSpeed.magnitude());
        }

        //clamping the target angular speed if a too large speed was provided
        if (Math.abs(unitAngularSpeed) > 1) {
            unitAngularSpeed = unitAngularSpeed / Math.abs(unitAngularSpeed);
        }

        Vector2d linearSpeed = linearRateLimiter.limit(unitLinearSpeed);
        double angularSpeed = angularRateLimiter.limit(unitAngularSpeed);

        //revolving the linear around the orientation to make the robot field centric
        Vector2d rotatedLinearSpeed = linearSpeed.rotateBy(getHeading().getDegrees());

        double absoluteMotionSum = Math.max(1,Math.abs(rotatedLinearSpeed.getX()) + Math.abs(rotatedLinearSpeed.getY()) + Math.abs(angularSpeed));

        double leftFrontVelocity = (rotatedLinearSpeed.getX() + rotatedLinearSpeed.getY() + angularSpeed) / absoluteMotionSum;
        double rightFrontVelocity = (rotatedLinearSpeed.getX() - rotatedLinearSpeed.getY() - angularSpeed) / absoluteMotionSum;
        double leftBackVelocity = (rotatedLinearSpeed.getX() - rotatedLinearSpeed.getY() + angularSpeed) / absoluteMotionSum;
        double rightBackVelocity = (rotatedLinearSpeed.getX() + rotatedLinearSpeed.getY() - angularSpeed) / absoluteMotionSum;

        leftFrontMotor.setVelocity(leftFrontVelocity * Constants.Drive.MOTOR_MAX_VELOCITY);
        rightFrontMotor.setVelocity(rightFrontVelocity * Constants.Drive.MOTOR_MAX_VELOCITY);
        leftBackMotor.setVelocity(leftBackVelocity * Constants.Drive.MOTOR_MAX_VELOCITY);
        rightBackMotor.setVelocity(rightBackVelocity * Constants.Drive.MOTOR_MAX_VELOCITY);

    }

    public Rotation2d getHeading() {
        double rawImuReading = imu.getRobotYawPitchRollAngles().getYaw();
        if (Constants.Core.IMU_REVERSED) {
            rawImuReading *= -1;
        }
        return Rotation2d.fromDegrees(rawImuReading).rotateBy(imuOffset);
    }

    public void setPositionFromVision(VisionLocalizationSubsystem.VisionLocalizationPoseResult poseResult) {
        if (poseResult.foundPose) {
            robotPosition = new Vector2d(poseResult.pose.getX(DistanceUnit.INCH),poseResult.pose.getY(DistanceUnit.INCH));
            if (haveCameraSetOrientation) {
                setIMU(new Rotation2d(poseResult.pose.getHeading(AngleUnit.RADIANS)).rotateBy(new Rotation2d(0.5 * Math.PI)));
            }
        }
    }

    public void setPose(Vector2d pose) {
        robotPosition = pose;
    }

    public void periodic() {

        //getting the motors changed values
        int leftFrontChange = leftFrontMotor.getCurrentPosition() - lastLeftFrontPosition;
        lastLeftFrontPosition = leftFrontMotor.getCurrentPosition();

        int rightFrontChange = rightFrontMotor.getCurrentPosition() - lastRightFrontPosition;
        lastRightFrontPosition = rightFrontMotor.getCurrentPosition();

        int leftBackChange = leftBackMotor.getCurrentPosition() - lastLeftBackPosition;
        lastLeftBackPosition = leftBackMotor.getCurrentPosition();

        int rightBackChange = rightBackMotor.getCurrentPosition() - lastRightBackPosition;
        lastRightBackPosition = rightBackMotor.getCurrentPosition();

        Vector2d positionChange = new Vector2d(
            leftFrontChange + rightFrontChange + leftBackChange + rightBackChange,
            -leftFrontChange + rightFrontChange + leftBackChange - rightBackChange
        ).rotateBy(getHeading().getDegrees()).times(Constants.Drive.DISPLACEMENT_PER_TICKS);
        robotPosition = robotPosition.plus(positionChange);

        Robot.getInstance().opMode.telemetry.addData("Position", robotPosition.toString());
        Robot.getInstance().opMode.telemetry.addData("Heading", getHeading().getRadians());

    }

    // A Builder class to facilitate the construction of the main class (presumably for a robot or drivetrain).
    // It allows users to configure the motors step by step in a clear and concise manner.
    public static class Builder {

        // Storing motor instances in the builder to eventually assign them to the main class.
        private DcMotorEx leftFrontMotor;    // Motor for the front-left wheel
        private DcMotorEx rightFrontMotor;   // Motor for the front-right wheel
        private DcMotorEx leftBackMotor;     // Motor for the back-left wheel
        private DcMotorEx rightBackMotor;    // Motor for the back-right wheel

        //imu
        private IMU imu;

        // Method to add and configure the left front motor.
        public Builder addLeftFrontMotor(DcMotorEx motor) {
            this.leftFrontMotor = motor;
            return this;  // Returning 'this' allows method chaining.
        }

        // Method to add and configure the right front motor.
        public Builder addRightFrontMotor(DcMotorEx motor) {
            this.rightFrontMotor = motor;
            return this;
        }

        // Method to add and configure the left back motor.
        public Builder addLeftBackMotor(DcMotorEx motor) {
            this.leftBackMotor = motor;
            return this;
        }

        // Method to add and configure the right back motor.
        public Builder addRightBackMotor(DcMotorEx motor) {
            this.rightBackMotor = motor;
            return this;
        }

        // Method to add and configure the IMU
        public Builder addIMU(IMU imu) {
            this.imu = imu;
            return this;
        }

        // Optionally, you can add a build method to create the final instance of the class you're building.
        public DriveSubsystem build() {
            return new DriveSubsystem(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor,imu);
        }
    }
}
