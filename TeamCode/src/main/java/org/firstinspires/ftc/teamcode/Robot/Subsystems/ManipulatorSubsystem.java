package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

public class ManipulatorSubsystem {

    Servo wrist;
    Servo claw;
    CRServo roller;

    public ManipulatorSubsystem(Servo wrist, Servo claw, CRServo roller) {
        this.wrist = wrist;
        this.claw = claw;
        this.roller = roller;
    }

    public void manipulateFromGampead(Gamepad gamepad, Constants.ArmPosition armPosition) {

        switch(armPosition) {
            case IDLE:
            case SPECIMEN_INTAKE:
            case CHAMBER_ONE:
            case CHAMBER_TWO:
                roller.setPower(Constants.Manipulator.ROLLER_STOPPED_SPEED);
                if (gamepad.left_bumper) {
                    claw.setPosition(Constants.Manipulator.CLAW_CLOSED_POSITION);
                }
                if (gamepad.right_bumper) {
                    claw.setPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                }
                break;
            case SAMPLE_INTAKE:
            case BASKET_ONE:
            case BASKET_TWO:

                claw.setPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                if (gamepad.left_bumper == gamepad.right_bumper) {
                    roller.setPower(Constants.Manipulator.ROLLER_STOPPED_SPEED);
                } else {
                    if (gamepad.left_bumper) {
                        roller.setPower(Constants.Manipulator.ROLLER_INTAKE_SPEED);
                    } else {
                        roller.setPower(Constants.Manipulator.ROLLER_OUTTAKE_SPEED);
                    }
                }
                break;
            default:
        }

    }

}
