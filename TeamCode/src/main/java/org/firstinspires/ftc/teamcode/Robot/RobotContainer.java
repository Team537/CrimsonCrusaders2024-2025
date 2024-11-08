package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ManipulatorSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.CommandBase;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

public class RobotContainer {

    //Stores the op mode (required to get controller data)
    LinearOpMode opMode;

    //Subsystems
    DriveSubsystem driveSubsystem;
    ArmSubsystem armSubsystem;
    ManipulatorSubsystem manipulatorSubsystem;



    /**
     * Creates the robot container to schedule saved commands
     * @param opMode the opMode object
     * @param opModeType the type of opMode
     */
    RobotContainer(LinearOpMode opMode, Constants.OpModes opModeType) {

        //defining opMode
        this.opMode = opMode;

        //defining subsystems
        driveSubsystem = new DriveSubsystem.Builder()
            .addLeftFrontMotor(opMode.hardwareMap.get(DcMotorEx.class, Constants.Drive.LEFT_FRONT_DRIVE_MOTOR_NAME))
            .addRightFrontMotor(opMode.hardwareMap.get(DcMotorEx.class, Constants.Drive.RIGHT_FRONT_DRIVE_MOTOR_NAME))
            .addLeftBackMotor(opMode.hardwareMap.get(DcMotorEx.class, Constants.Drive.LEFT_BACK_DRIVE_MOTOR_NAME))
            .addRightBackMotor(opMode.hardwareMap.get(DcMotorEx.class, Constants.Drive.RIGHT_BACK_DRIVE_MOTOR_NAME))
            .addIMU(opMode.hardwareMap.get(IMU.class, Constants.Core.IMU_NAME))
            .build();

        armSubsystem = new ArmSubsystem(
            opMode.hardwareMap.get(DcMotorEx.class, Constants.Arm.SHOULDER_MOTOR_NAME),
            opMode.hardwareMap.get(DcMotorEx.class, Constants.Arm.LINEAR_SLIDE_MOTOR_NAME)
        );

        manipulatorSubsystem = new ManipulatorSubsystem(
            opMode.hardwareMap.get(Servo.class, Constants.Manipulator.WRIST_SERVO_NAME),
            opMode.hardwareMap.get(Servo.class, Constants.Manipulator.CLAW_SERVO_NAME),
            opMode.hardwareMap.get(CRServo.class, Constants.Manipulator.ROLLER_SERVO_NAME)
        );

        //schedule different commands depending on the opMode
        switch (opModeType) {

            //manual driving command
            case TELE_OP:


                break;
        }
    }

    public void scheduleCommand() {
        getManualDriveSubsystemCommand().schedule();
        getManualArmSubsystemCommand().schedule();
        //getManualManipulatorSubsystemCommand().schedule();
    }

    private CommandBase getManualDriveSubsystemCommand() {
        return new RunCommand(
            () -> {
                driveSubsystem.driveFromGamepad(opMode.gamepad1);
            }
        );
    }

    private CommandBase getManualArmSubsystemCommand() {
        return new RunCommand(
            () -> {
                armSubsystem.armFromGamepad(opMode.gamepad1);
            }
        );
    }

    private CommandBase getManualManipulatorSubsystemCommand() {
        return new RunCommand(
            () -> {
                manipulatorSubsystem.manipulateFromGampead(opMode.gamepad1,armSubsystem.getArmPosition());
            }
        );
    }

}
