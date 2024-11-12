package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.RateLimiter;
import org.firstinspires.ftc.teamcode.Utilities.RateLimiter2D;

public class DriveSubsystem extends Subsystem {

    //saving motors
    private DcMotorEx leftFrontMotor;
    private DcMotorEx rightFrontMotor;
    private DcMotorEx leftBackMotor;
    private DcMotorEx rightBackMotor;
    private IMU imu;

    private RateLimiter2D linearRateLimiter;
    private RateLimiter angularRateLimiter;

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
            -gamepad.left_stick_y
        );
        double angularSpeed = gamepad.right_stick_x;
        if (gamepad.guide) {
            imu.resetYaw();
        }
        drive(linearSpeed.times(Constants.Drive.MAX_LINEAR_SPEED),angularSpeed*Constants.Drive.MAX_ANGULAR_SPEED);
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
    public void drive(Vector2d targetLinearSpeed, double targetAngularSpeed) {

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

        //the motor inputs will be too high if the sum of the inputs are greater than one
        if (unitLinearSpeed.magnitude() + Math.abs(unitAngularSpeed) > 1.0) {

            //this variable needs to be saved as the normal unitAngularSpeed gets modified
            double oldUnitAngularSpeed = unitAngularSpeed;
            unitAngularSpeed *= (Constants.Drive.TURN_OVERRIDE * Math.abs(unitAngularSpeed) ) + ( (1 - Constants.Drive.TURN_OVERRIDE) * (1 - unitLinearSpeed.magnitude()) );
            unitLinearSpeed = unitLinearSpeed.times( ( Constants.Drive.TURN_OVERRIDE * (1 - Math.abs(oldUnitAngularSpeed)) ) + ( (1 - Constants.Drive.TURN_OVERRIDE) * (unitLinearSpeed.magnitude()) ) );
        }

        Vector2d linearSpeed = linearRateLimiter.limit(unitLinearSpeed);
        double angularSpeed = angularRateLimiter.limit(unitAngularSpeed);

        //revolving the linear around the orientation to make the robot field centric
        Vector2d rotatedLinearSpeed = linearSpeed.rotateBy(getHeading().getDegrees());

        //calculating the values used for linear velocity of the motors
        double cosineMove = ((rotatedLinearSpeed.getX() - rotatedLinearSpeed.getY())/Math.sqrt(2));
        double sineMove = ((rotatedLinearSpeed.getX() + rotatedLinearSpeed.getY())/Math.sqrt(2));

        double cosinePivot;
        double sinePivot;

        double mag = new Vector2d(cosineMove,sineMove).magnitude();
        double angle = Math.atan2(sineMove,cosineMove);

        //setting the rotation speeds to the right stick x variable multiplied by a changing function which ensures that the motor speed can't exceed 0, resulting in smoother movement
        cosinePivot = angularSpeed * ((1 - mag) + 2 * mag * Math.pow(Math.cos(angle),2));
        sinePivot = angularSpeed * ((1 - mag) + 2 * mag * Math.pow(Math.sin(angle),2));

        double leftFrontVelocity = cosineMove + cosinePivot;
        double rightFrontVelocity = sineMove - sinePivot;
        double leftBackVelocity = sineMove + sinePivot;
        double rightBackVelocity = cosineMove - cosinePivot;

        leftFrontMotor.setVelocity(Constants.Drive.MOTOR_MAX_VELOCITY * leftFrontVelocity);
        rightFrontMotor.setVelocity(Constants.Drive.MOTOR_MAX_VELOCITY * rightFrontVelocity);
        leftBackMotor.setVelocity(Constants.Drive.MOTOR_MAX_VELOCITY * leftBackVelocity);
        rightBackMotor.setVelocity(Constants.Drive.MOTOR_MAX_VELOCITY * rightBackVelocity);

    }

    public Rotation2d getHeading() {
        double imuReading = imu.getRobotYawPitchRollAngles().getYaw();
        if (Constants.Core.IMU_REVERSED) {
            imuReading *= -1;
        }
        return Rotation2d.fromDegrees(imuReading);
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
