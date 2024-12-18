package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.PID;
import org.firstinspires.ftc.teamcode.Utilities.RateLimiter;
import org.firstinspires.ftc.teamcode.Utilities.RateLimiter2D;

public class ArmSubsystem extends Subsystem {

    //Saving Motors
    DcMotorEx shoulderMotor;
    DcMotorEx linearSlideMotor;

    double shoulderPower = 0.0;
    double linearSlidePower;
    int linearSlidePosition = 0;

    Constants.ArmPosition armPosition = Constants.ArmPosition.IDLE;
    Constants.ArmPosition previousArmPosition = Constants.ArmPosition.IDLE;

    PID shoulderPID = new PID(Constants.Arm.SHOULDER_KP,Constants.Arm.SHOULDER_KI,Constants.Arm.SHOULDER_KD);
    PID linearSlidePID = new PID(Constants.Arm.LINEAR_SLIDE_KP,Constants.Arm.LINEAR_SLIDE_KI,Constants.Arm.LINEAR_SLIDE_KD);

    //Constructor Class for creating an arm subsystem
    public ArmSubsystem(
        DcMotorEx shoulderMotor,
        DcMotorEx linearSlideMotor
    ) {
        this.shoulderMotor = shoulderMotor;     // Assign the left front motor to the instance
        this.linearSlideMotor = linearSlideMotor;   // Assign the right front motor to the instance

        shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * sets the arm positions to zero
     */
    public void resetArmPositions() {
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * gets the current target position
     * @return the current target arm position
     */
    public Constants.ArmPosition getArmPosition() {
        return armPosition;
    }

    /**
     * Sets the arms goal position to buttons on the gamepad
     * @param gamepad the gamepad to drive with
     */
    public void armFromGamepad(Gamepad gamepad) {

        // Setting the arm position based on gamepad input

        // Setting the idle position
        if (gamepad.dpad_down) {
            armPosition = Constants.ArmPosition.IDLE;
        }

        // Setting the sample intake position
        else if (gamepad.dpad_left) {
            armPosition = Constants.ArmPosition.SAMPLE_INTAKE;
        }

        // Setting the specimen intake position
        else if (gamepad.dpad_right) {
            armPosition = Constants.ArmPosition.SPECIMEN_INTAKE;
        }

        // Setting the basket one position
        else if (gamepad.a) {
            armPosition = Constants.ArmPosition.BASKET_ONE;
        }

        // Setting the basket two position
        else if (gamepad.b) {
            armPosition = Constants.ArmPosition.BASKET_TWO;
        }

        // Setting the chamber one position
        else if (gamepad.x) {
            armPosition = Constants.ArmPosition.CHAMBER_ONE;
        }

        // Setting the chamber two position
        else if (gamepad.y) {
            armPosition = Constants.ArmPosition.CHAMBER_TWO;
        }

        // Setting the climb position
        else if (gamepad.dpad_up) {
            armPosition = Constants.ArmPosition.CLIMB;
        }

    }

    public void setArmPosition(Constants.ArmPosition armPosition) {
        this.armPosition = armPosition;
    }

    public void armPowerTest(Gamepad gamepad) {

        if (gamepad.left_bumper) {
            shoulderPower += 0.001;
        }
        if (gamepad.right_bumper) {
            shoulderPower -= 0.001;
        }
        if (gamepad.left_trigger > 0.9) {
            linearSlidePower += 0.001;
        }
        if (gamepad.right_trigger > 0.9) {
            linearSlidePower -= 0.001;
        }

        if (gamepad.a) {
            shoulderMotor.setPower(shoulderPower);
        } else {
            shoulderMotor.setPower(0);
        }

        if (gamepad.b) {
            linearSlideMotor.setPower(linearSlidePower);
        } else {
            linearSlideMotor.setPower(0);
        }

        periodic();
    }


    public int getShoulderPosition() {
        return shoulderMotor.getCurrentPosition();
    }
    public int getLinearSlidePosition() {
        return linearSlideMotor.getCurrentPosition();
    }

    public void periodic() {

        //Only update the proper arm system if the arm is fully retracted. This means that the shoulder won't go down until it is fully restracted
        if (Math.abs(Constants.ArmPosition.IDLE.LINEAR_SLIDE_POSITION - linearSlideMotor.getCurrentPosition()) < 200 || Math.abs(armPosition.SHOULDER_POSITION - shoulderMotor.getCurrentPosition()) < 40) {
            previousArmPosition = armPosition;
        }

        double armFValue;
        //find the feedforward value, will be used later
        if (armPosition == Constants.ArmPosition.BASKET_TWO || shoulderMotor.getCurrentPosition() < 750) {
            armFValue = Constants.Arm.getArmFValue(shoulderMotor.getCurrentPosition(), linearSlideMotor.getCurrentPosition());
        } else {
            armFValue = 0.0;
        }

        shoulderMotor.setPower(
            (1 - armFValue) * Math.tanh(shoulderPID.calculate(previousArmPosition.SHOULDER_POSITION - shoulderMotor.getCurrentPosition())) + armFValue
        );

        //Set the linear slide to retract if the shoulder hasn't reached it's position yet
        if (Math.abs(armPosition.SHOULDER_POSITION - shoulderMotor.getCurrentPosition()) < 40) {
            linearSlideMotor.setPower(Math.tanh(
                linearSlidePID.calculate(armPosition.LINEAR_SLIDE_POSITION - linearSlideMotor.getCurrentPosition())
            ));
        } else {
            linearSlideMotor.setPower(Math.tanh(
                linearSlidePID.calculate(Constants.ArmPosition.IDLE.LINEAR_SLIDE_POSITION - linearSlideMotor.getCurrentPosition())
            ));
        }


        Robot.getInstance().opMode.telemetry.addData("Shoulder Position",shoulderMotor.getCurrentPosition());
        Robot.getInstance().opMode.telemetry.addData("Linear Slide Position",linearSlideMotor.getCurrentPosition());

        Robot.getInstance().opMode.telemetry.addData("Commanded Shoulder Power",shoulderPower);
        Robot.getInstance().opMode.telemetry.addData("Commanded Linear Slide Power",linearSlidePower);

        Robot.getInstance().opMode.telemetry.addData("Shoulder Power",shoulderMotor.getPower());
        Robot.getInstance().opMode.telemetry.addData("Linear Slide Power",linearSlideMotor.getPower());

    }


}
