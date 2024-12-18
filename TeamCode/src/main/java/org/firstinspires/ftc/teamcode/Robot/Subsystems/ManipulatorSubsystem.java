package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.Subsystem;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

public class ManipulatorSubsystem extends Subsystem {

    Servo wrist;
    Servo claw;
    CRServo roller;

    boolean rollerLastSetToIntake = false;
    boolean doubleButtonLastStatus = false;
    boolean doubleButtonToggle = false;
    boolean doubleButtonActive = false;
    Constants.ArmPosition lastArmPosition = Constants.ArmPosition.IDLE;

    double rollerChangedTimestamp = Robot.getInstance().opMode.time;

    public ManipulatorSubsystem(Servo wrist, Servo claw, CRServo roller) {
        this.wrist = wrist;
        this.claw = claw;
        this.roller = roller;
    }

    //Control the rollers using the bumpers on the gamepad
    public void controlRollerFromGamepad(Gamepad gamepad) {

        //if both values are held, or neither, then stop the rollers
        if (gamepad.left_bumper == gamepad.right_bumper) {
            if (rollerLastSetToIntake) {
                setRollerPower(Constants.Manipulator.ROLLER_HOLD_SPEED);
            } else {
                setRollerPower(Constants.Manipulator.ROLLER_STOPPED_SPEED);
            }
        } else {
            if (gamepad.left_bumper) {
                rollerLastSetToIntake = true;
                setRollerPower(Constants.Manipulator.ROLLER_INTAKE_SPEED);
            } else {
                rollerLastSetToIntake = false;
                setRollerPower(Constants.Manipulator.ROLLER_OUTTAKE_SPEED);
            }
        }
    }

    public void manipulateFromGampead(Gamepad gamepad, Constants.ArmPosition armPosition, int shoulderPosition, int linearSlidePosition) {

        //setting some defaults upon manipulator changes
        if (lastArmPosition != armPosition) {
            lastArmPosition = armPosition;
            doubleButtonLastStatus = false;
            doubleButtonToggle = false;
            doubleButtonActive = false;

            switch (armPosition) {
                case CHAMBER_ONE:
                    wrist.setPosition(Constants.Manipulator.HOVER_CHAMBER_ONE_WRIST_POSITION);
                    break;
                case CHAMBER_TWO:
                    wrist.setPosition(Constants.Manipulator.HOVER_CHAMBER_TWO_WRIST_POSITION);
                    break;
            }
        }

        //Controlling the intakes based on the arm position
        switch (armPosition) {
            case IDLE:

                //Control both the claw and the rollers
                if (gamepad.left_bumper) {
                    claw.setPosition(Constants.Manipulator.CLAW_CLOSED_POSITION);
                }
                if (gamepad.right_bumper) {
                    claw.setPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                }

                controlRollerFromGamepad(gamepad);
                if (shoulderPosition < 750) {
                    wrist.setPosition(Constants.Manipulator.IDLE_WRIST_POSITION);
                } else {
                    wrist.setPosition(Constants.Manipulator.BASKET_TWO_WRIST_POSITION);
                }

                break;

            case SAMPLE_INTAKE:

                claw.setPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                controlRollerFromGamepad(gamepad);
                if (Math.abs(linearSlidePosition - Constants.ArmPosition.SAMPLE_INTAKE.LINEAR_SLIDE_POSITION) < 150) {
                    wrist.setPosition(Constants.Manipulator.SAMPLE_INTAKE_WRIST_POSITION);
                } else {
                    wrist.setPosition(Constants.Manipulator.IDLE_WRIST_POSITION);
                }

                break;

            case SPECIMEN_INTAKE:

                setRollerPower(Constants.Manipulator.ROLLER_STOPPED_SPEED);

                //Control the claw (if left bumper double pressed, retract the wrist)
                if (gamepad.left_bumper) {
                    claw.setPosition(Constants.Manipulator.CLAW_CLOSED_POSITION);

                    if (!doubleButtonLastStatus && doubleButtonToggle) {
                        doubleButtonActive = true;
                    }
                    doubleButtonLastStatus = true;
                    doubleButtonToggle = true;
                } else {
                    doubleButtonLastStatus = false;
                }
                if (gamepad.right_bumper) {
                    claw.setPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                    doubleButtonActive = false;
                    doubleButtonToggle = false;
                }

                //Pull the wrist up when the button is double clicked
                if (doubleButtonActive) {
                    wrist.setPosition(Constants.Manipulator.RETRACT_SPECIMEN_INTAKE_WRIST_POSITION);
                } else {
                    wrist.setPosition(Constants.Manipulator.GRAB_SPECIMEN_INTAKE_WRIST_POSITION);
                }

                break;

            case BASKET_ONE:

                wrist.setPosition(Constants.Manipulator.BASKET_ONE_WRIST_POSITION);
                controlRollerFromGamepad(gamepad);
                break;

            case BASKET_TWO:

                wrist.setPosition(Constants.Manipulator.BASKET_TWO_WRIST_POSITION);
                controlRollerFromGamepad(gamepad);
                break;

            case CHAMBER_ONE:

                setRollerPower(Constants.Manipulator.ROLLER_STOPPED_SPEED);

                //Control the wrist (if left bumper double pressed, place the specimen)
                if (gamepad.left_bumper) {
                    wrist.setPosition(Constants.Manipulator.HOVER_CHAMBER_ONE_WRIST_POSITION);
                    doubleButtonActive = false;
                    doubleButtonToggle = false;
                }
                if (gamepad.right_bumper) {
                    wrist.setPosition(Constants.Manipulator.PLACE_CHAMBER_ONE_WRIST_POSITION);
                    if (!doubleButtonLastStatus && doubleButtonToggle) {
                        doubleButtonActive = true;
                    }
                    doubleButtonLastStatus = true;
                    doubleButtonToggle = true;
                } else {
                    doubleButtonLastStatus = false;
                }

                if (doubleButtonActive) {
                    claw.setPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                } else {
                    claw.setPosition(Constants.Manipulator.CLAW_CLOSED_POSITION);
                }

                break;

            case CHAMBER_TWO:

                setRollerPower(Constants.Manipulator.ROLLER_STOPPED_SPEED);

                //Control the wrist (if left bumper double pressed, place the specimen)
                if (gamepad.left_bumper) {
                    wrist.setPosition(Constants.Manipulator.HOVER_CHAMBER_TWO_WRIST_POSITION);
                    doubleButtonActive = false;
                    doubleButtonToggle = false;
                }
                if (gamepad.right_bumper) {
                    wrist.setPosition(Constants.Manipulator.PLACE_CHAMBER_TWO_WRIST_POSITION);
                    
                    if (!doubleButtonLastStatus && doubleButtonToggle) {
                        doubleButtonActive = true;
                    }
                    doubleButtonLastStatus = true;
                    doubleButtonToggle = true;
                } else {
                    doubleButtonLastStatus = false;
                }

                if (doubleButtonActive) {
                    claw.setPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                } else {
                    claw.setPosition(Constants.Manipulator.CLAW_CLOSED_POSITION);
                }

                break;

            case CLIMB:

                wrist.setPosition(Constants.Manipulator.CLIMB_WRIST_POSITION);
                setRollerPower(Constants.Manipulator.ROLLER_STOPPED_SPEED);
                claw.setPosition(Constants.Manipulator.CLAW_CLOSED_POSITION);

                break;
        }

    }

    public void wristLimitsTest(Gamepad gamepad) {
        wrist.setPosition(wrist.getPosition() + gamepad.left_trigger * 0.01);
        wrist.setPosition(wrist.getPosition() - gamepad.right_trigger * 0.01);
        Robot.getInstance().opMode.telemetry.addData("Wrist Position",wrist.getPosition());
    }

    /**
     * sets the position of the wrist
     * @param position the position to set to
     */
    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }

    /**
     * sets the position of the claw
     * @param position the position to set to
     */
    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    /**
     * sets the power of the roller
     * @param power the power to set to
     */
    public void setRollerPower(double power) {
        rollerChangedTimestamp = Robot.getInstance().opMode.time;
        roller.setPower(power);
    }

    /**
     * gets the power of the roller
     * @return the power of the roller
     */
    public double getRollerPower() {
        return roller.getPower();
    }

    /**
     * returns the time that the rollers have been changed for, which is used for time-based autonomous features
     * @return the time, in seconds, that the rollers have been running
     */
    public double getTimeSinceRollerChanged() {
        return Robot.getInstance().opMode.time - rollerChangedTimestamp;
    }



}
