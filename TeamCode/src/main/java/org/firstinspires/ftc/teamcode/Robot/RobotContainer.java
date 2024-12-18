package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.Commands.InitializeAndRunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.InitializeRunAndFinishCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.OneTimeCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.Robot.Commands.RunCommand;
import org.firstinspires.ftc.teamcode.Robot.Commands.SelectableCommandGroup;
import org.firstinspires.ftc.teamcode.Robot.Commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ManipulatorSubsystem;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.RobotVision.VisionLocalizationSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.CommandBase;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Sample;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.function.Supplier;

public class RobotContainer {

    //Stores the op mode (required to get controller data)
    LinearOpMode opMode;

    //Subsystems
    DriveSubsystem driveSubsystem;
    ArmSubsystem armSubsystem;
    ManipulatorSubsystem manipulatorSubsystem;
    VisionLocalizationSubsystem visionLocalizationSubsystem;

    //All drive subsystem commands
    CommandBase manualDriveSubsystemCommand;
    CommandBase manualDriveSpecimenIntakeCommand;
    CommandBase manualDriveBasketOneCommand;
    CommandBase manualDriveBasketTwoCommand;
    CommandBase manualDriveChamberOneCommand;
    CommandBase manualDriveChamberTwoCommand;
    CommandBase manualArmSubsystemCommand;
    CommandBase manualManipulatorSubsystemCommand;
    CommandBase visionLocalizationCommand;


    //Stores the current used Drive Command
    CommandBase activeDriveCommand;

    //saved data that is often used in the code
    public Alliance alliance = Alliance.RED;


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

        visionLocalizationSubsystem = new VisionLocalizationSubsystem(
            opMode.hardwareMap.get(WebcamName.class, Constants.Vision.Localization.LOCALIZATION_CAMERA_NAME)
        );

        manualDriveSubsystemCommand = getManualDriveSubsystemCommand();
        manualArmSubsystemCommand = getManualArmSubsystemCommand();
        manualManipulatorSubsystemCommand = getManualManipulatorSubsystemCommand();
        visionLocalizationCommand = getVisionLocalizationCommand();
        activeDriveCommand = manualDriveSubsystemCommand;

    }


    public void scheduleTeleOpCommands() {

        driveSubsystem.schedule();
        driveSubsystem.setIMU(new Rotation2d(0));
        armSubsystem.schedule();
        manipulatorSubsystem.schedule();
        visionLocalizationSubsystem.schedule();
        manualDriveSubsystemCommand.schedule();
        manualArmSubsystemCommand.schedule();
        manualManipulatorSubsystemCommand.schedule();
        visionLocalizationCommand.schedule();
        armSubsystem.resetArmPositions();

    }

    public void preMatchInit() {
        armSubsystem.resetArmPositions();
    }

    boolean allianceToggle = false;

    /**
     * The settings ran by the second controller to manipulate certain things before auto
     */
    public void settings() {

        //toggle alliance on the a button being selected
        if (opMode.gamepad2.a) {
            if (!allianceToggle) {

                //toggle alliance RED/BLUE
                if (alliance == Alliance.RED) {
                    alliance = Alliance.BLUE;
                } else {
                    alliance = Alliance.RED;
                }

            }
            allianceToggle = true;
        } else {
            allianceToggle = false;
        }

        if (alliance == Alliance.RED) {
            opMode.telemetry.addData("Alliance","RED");
        } else {
            opMode.telemetry.addData("Alliance","BLUE");
        }

    }

    private CommandBase getManualDriveSubsystemCommand() {
        return new RunCommand(
            () -> {
                driveSubsystem.driveFromGamepad(opMode.gamepad1);
            }
        );
    }

    private CommandBase getManualDriveSpecimenIntakeCommand() {
        return new InitializeAndRunCommand(
            () -> {
                driveSubsystem.resetSpecimenIntakePosition();
            },
            () -> {
                driveSubsystem.specimenIntakeFromGamepad(opMode.gamepad1);
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
                manipulatorSubsystem.manipulateFromGampead(opMode.gamepad1,armSubsystem.getArmPosition(),armSubsystem.getShoulderPosition(),armSubsystem.getLinearSlidePosition());
            }
        );

    }

    private CommandBase getVisionLocalizationCommand() {
        return new RunCommand(
            () -> {
                driveSubsystem.setPositionFromVision(visionLocalizationSubsystem.getPose());
            }
        );
    }

    //The command used for automated scoring
    public void scheduleAutonomousCommands() {

        LinkedList<CommandBase> commandList = new LinkedList<>();

        InitializeRunAndFinishCommand scoreSampleCommand = new InitializeRunAndFinishCommand(
            () -> {
                //put the correct set positions in
                if (alliance == Alliance.RED) {
                    driveSubsystem.setTargetPosition(Constants.Arena.RED_SAMPLE_SCORE_POSE);
                } else {
                    driveSubsystem.setTargetPosition(Constants.Arena.BLUE_SAMPLE_SCORE_POSE);
                }
                armSubsystem.setArmPosition(Constants.ArmPosition.BASKET_TWO);
                manipulatorSubsystem.setWristPosition(Constants.Manipulator.BASKET_TWO_WRIST_POSITION);
                manipulatorSubsystem.setRollerPower(Constants.Manipulator.ROLLER_HOLD_SPEED);
            },
            () -> {

                //Drive to the position and set the arm to the correct height, while also positioning the wrist
                driveSubsystem.driveToPosition(Constants.Drive.MAX_LINEAR_SPEED,Constants.Drive.MAX_ANGULAR_SPEED);

                //if the robot and arm are at the correct positions, release the sample
                if (
                    driveSubsystem.getDistanceToPose() < Constants.Drive.MINIMUM_TARGET_LINEAR_ERROR &&
                        driveSubsystem.getAngleToPose() < Constants.Drive.MINIMUM_TARGET_ANGULAR_ERROR &&
                        Math.abs(Constants.ArmPosition.BASKET_TWO.SHOULDER_POSITION - armSubsystem.getShoulderPosition()) < Constants.Arm.SHOULDER_MINIMUM_ERROR &&
                        Math.abs(Constants.ArmPosition.BASKET_TWO.LINEAR_SLIDE_POSITION - armSubsystem.getLinearSlidePosition()) < Constants.Arm.LINEAR_SLIDE_MINIMUM_ERROR &&
                        Math.abs(manipulatorSubsystem.getRollerPower() - Constants.Manipulator.ROLLER_OUTTAKE_SPEED) > 1e-4
                ) {
                    manipulatorSubsystem.setRollerPower(Constants.Manipulator.ROLLER_OUTTAKE_SPEED);
                }

            },
            () -> {
                if (
                    driveSubsystem.getDistanceToPose() < Constants.Drive.MINIMUM_TARGET_LINEAR_ERROR &&
                        driveSubsystem.getAngleToPose() < Constants.Drive.MINIMUM_TARGET_ANGULAR_ERROR &&
                        Math.abs(Constants.ArmPosition.BASKET_TWO.SHOULDER_POSITION - armSubsystem.getShoulderPosition()) < Constants.Arm.SHOULDER_MINIMUM_ERROR &&
                        Math.abs(Constants.ArmPosition.BASKET_TWO.LINEAR_SLIDE_POSITION - armSubsystem.getLinearSlidePosition()) < Constants.Arm.LINEAR_SLIDE_MINIMUM_ERROR &&
                        manipulatorSubsystem.getTimeSinceRollerChanged() > Constants.Manipulator.ROLLER_OUTTAKE_MINIMUM_TIME &&
                        Math.abs(manipulatorSubsystem.getRollerPower() - Constants.Manipulator.ROLLER_OUTTAKE_SPEED) < 1e-4
                ) {
                    return true;
                } else {
                    return false;
                }
            }
        );

        InitializeRunAndFinishCommand idleAfterScoreSampleCommand = new InitializeRunAndFinishCommand(
            () -> {

                armSubsystem.setArmPosition(Constants.ArmPosition.IDLE);
                manipulatorSubsystem.setWristPosition(Constants.Manipulator.IDLE_WRIST_POSITION);
                manipulatorSubsystem.setRollerPower(Constants.Manipulator.ROLLER_STOPPED_SPEED);
                driveSubsystem.setTargetPosition(Constants.Arena.RED_SAMPLE_ONE_LINE_UP_POSE);
            },
            () -> {
                driveSubsystem.driveToPosition(Constants.Drive.MAX_LINEAR_SPEED,Constants.Drive.MAX_ANGULAR_SPEED);
            },
            () -> {
                if (
                        Math.abs(Constants.ArmPosition.IDLE.SHOULDER_POSITION - armSubsystem.getShoulderPosition()) < Constants.Arm.SHOULDER_MINIMUM_ERROR &&
                        Math.abs(Constants.ArmPosition.IDLE.LINEAR_SLIDE_POSITION - armSubsystem.getLinearSlidePosition()) < Constants.Arm.LINEAR_SLIDE_MINIMUM_ERROR
                ) {
                    return true;
                } else {
                    return false;
                }
            }
        );

        //Initializing the robot so that it is ready to move
        commandList.add(
            new InitializeRunAndFinishCommand(
                () -> {
                    driveSubsystem.setHaveCameraSetOrientation(true);
                    driveSubsystem.drive(new Vector2d(0, 0), 0);
                    armSubsystem.setArmPosition(Constants.ArmPosition.IDLE);
                    manipulatorSubsystem.setClawPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                    manipulatorSubsystem.setRollerPower(Constants.Manipulator.ROLLER_HOLD_SPEED);
                },
                () -> {

                },
                () -> {
                    if (manipulatorSubsystem.getTimeSinceRollerChanged() > 2.0) {
                        return true;
                    } else {
                        return false;
                    }
                }
            )
        );

        commandList.add(
            new InitializeRunAndFinishCommand(
                () -> {
                    //put the correct set positions in
                    if (alliance == Alliance.RED) {
                        driveSubsystem.setTargetPosition(Constants.Arena.RED_SAMPLE_SCORE_SEEK_POSE);
                    } else {
                        driveSubsystem.setTargetPosition(Constants.Arena.BLUE_SAMPLE_SCORE_SEEK_POSE);
                    }
                    driveSubsystem.setHaveCameraSetOrientation(false);
                },
                () -> {
                    driveSubsystem.driveToPosition(Constants.Drive.MAX_LINEAR_SPEED,Constants.Drive.MAX_ANGULAR_SPEED);
                },
                () -> {
                    if (
                        driveSubsystem.getDistanceToPose() < Constants.Drive.MINIMUM_TARGET_LINEAR_ERROR &&
                            driveSubsystem.getAngleToPose() < Constants.Drive.MINIMUM_TARGET_ANGULAR_ERROR
                    ) {
                        return true;
                    } else {
                        return false;
                    }
                }
            )
        );

        //Driving to the net zone and scoring a sample
        commandList.add(
            scoreSampleCommand
        );

        commandList.add(
            idleAfterScoreSampleCommand
        );

        /*
        commandList.add(
            new InitializeRunAndFinishCommand(
                () -> {
                    // Determine target pose based on alliance and sample enum
                    Pose2D targetPose;
                    targetPose = (alliance == Alliance.RED)
                        ? Constants.Arena.RED_SAMPLE_ONE_LINE_UP_POSE
                        : Constants.Arena.BLUE_SAMPLE_ONE_LINE_UP_POSE;

                    driveSubsystem.setTargetPosition(targetPose);

                    armSubsystem.setArmPosition(Constants.ArmPosition.SAMPLE_INTAKE);
                    manipulatorSubsystem.setWristPosition(Constants.Manipulator.SAMPLE_INTAKE_WRIST_POSITION);
                    manipulatorSubsystem.setClawPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                    manipulatorSubsystem.setRollerPower(Constants.Manipulator.ROLLER_INTAKE_SPEED);
                },
                () -> {
                    driveSubsystem.driveToPosition(Constants.Drive.MAX_LINEAR_SPEED,Constants.Drive.MAX_ANGULAR_SPEED);
                },
                () -> {
                    if (
                        driveSubsystem.getDistanceToPose() < Constants.Drive.MINIMUM_TARGET_LINEAR_ERROR &&
                            driveSubsystem.getAngleToPose() < Constants.Drive.MINIMUM_TARGET_ANGULAR_ERROR &&
                            Math.abs(Constants.ArmPosition.SAMPLE_INTAKE.SHOULDER_POSITION - armSubsystem.getShoulderPosition()) < Constants.Arm.SHOULDER_MINIMUM_ERROR &&
                            Math.abs(Constants.ArmPosition.SAMPLE_INTAKE.LINEAR_SLIDE_POSITION - armSubsystem.getLinearSlidePosition()) < Constants.Arm.LINEAR_SLIDE_MINIMUM_ERROR
                    ) {
                        return true;
                    } else {
                        return false;
                    }
                }
            )
        );

        commandList.add(
            new InitializeRunAndFinishCommand(
                () -> {
                    // Determine target pose based on alliance and sample enum
                    Pose2D targetPose;
                    targetPose = (alliance == Alliance.RED)
                        ? Constants.Arena.RED_SAMPLE_ONE_INTAKE_POSE
                        : Constants.Arena.BLUE_SAMPLE_ONE_INTAKE_POSE;

                    // Set the target position
                    driveSubsystem.setTargetPosition(targetPose);
                },
                () -> {
                    driveSubsystem.driveToPosition(0.25 * Constants.Drive.MAX_LINEAR_SPEED, 0.25 * Constants.Drive.MAX_ANGULAR_SPEED);
                },
                () -> {
                    if (
                        driveSubsystem.getDistanceToPose() < Constants.Drive.MINIMUM_TARGET_LINEAR_ERROR &&
                            driveSubsystem.getAngleToPose() < Constants.Drive.MINIMUM_TARGET_ANGULAR_ERROR
                    ) {
                        return true;
                    } else {
                        return false;
                    }
                }
            )
        );

        //Driving to the net zone and scoring a sample
        commandList.add(
            scoreSampleCommand
        );

        commandList.add(
            idleAfterScoreSampleCommand
        );

        commandList.add(
            new InitializeRunAndFinishCommand(
                () -> {
                    // Determine target pose based on alliance and sample enum
                    Pose2D targetPose;
                    targetPose = (alliance == Alliance.RED)
                        ? Constants.Arena.RED_SAMPLE_TWO_LINE_UP_POSE
                        : Constants.Arena.BLUE_SAMPLE_TWO_LINE_UP_POSE;

                    driveSubsystem.setTargetPosition(targetPose);

                    armSubsystem.setArmPosition(Constants.ArmPosition.SAMPLE_INTAKE);
                    manipulatorSubsystem.setWristPosition(Constants.Manipulator.SAMPLE_INTAKE_WRIST_POSITION);
                    manipulatorSubsystem.setClawPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                    manipulatorSubsystem.setRollerPower(Constants.Manipulator.ROLLER_INTAKE_SPEED);
                },
                () -> {
                    driveSubsystem.driveToPosition(Constants.Drive.MAX_LINEAR_SPEED,Constants.Drive.MAX_ANGULAR_SPEED);
                },
                () -> {
                    if (
                        driveSubsystem.getDistanceToPose() < Constants.Drive.MINIMUM_TARGET_LINEAR_ERROR &&
                            driveSubsystem.getAngleToPose() < Constants.Drive.MINIMUM_TARGET_ANGULAR_ERROR &&
                            Math.abs(Constants.ArmPosition.SAMPLE_INTAKE.SHOULDER_POSITION - armSubsystem.getShoulderPosition()) < Constants.Arm.SHOULDER_MINIMUM_ERROR &&
                            Math.abs(Constants.ArmPosition.SAMPLE_INTAKE.LINEAR_SLIDE_POSITION - armSubsystem.getLinearSlidePosition()) < Constants.Arm.LINEAR_SLIDE_MINIMUM_ERROR
                    ) {
                        return true;
                    } else {
                        return false;
                    }
                }
            )
        );

        commandList.add(
            new InitializeRunAndFinishCommand(
                () -> {
                    // Determine target pose based on alliance and sample enum
                    Pose2D targetPose;
                    targetPose = (alliance == Alliance.RED)
                        ? Constants.Arena.RED_SAMPLE_TWO_INTAKE_POSE
                        : Constants.Arena.BLUE_SAMPLE_TWO_INTAKE_POSE;

                    // Set the target position
                    driveSubsystem.setTargetPosition(targetPose);
                },
                () -> {
                    driveSubsystem.driveToPosition(0.25 * Constants.Drive.MAX_LINEAR_SPEED, 0.25 * Constants.Drive.MAX_ANGULAR_SPEED);
                },
                () -> {
                    if (
                        driveSubsystem.getDistanceToPose() < Constants.Drive.MINIMUM_TARGET_LINEAR_ERROR &&
                            driveSubsystem.getAngleToPose() < Constants.Drive.MINIMUM_TARGET_ANGULAR_ERROR
                    ) {
                        return true;
                    } else {
                        return false;
                    }
                }
            )
        );

        //Driving to the net zone and scoring a sample
        commandList.add(
            scoreSampleCommand
        );

        commandList.add(
            idleAfterScoreSampleCommand
        );

        commandList.add(
            new InitializeRunAndFinishCommand(
                () -> {
                    // Determine target pose based on alliance and sample enum
                    Pose2D targetPose;
                    targetPose = (alliance == Alliance.RED)
                        ? Constants.Arena.RED_SAMPLE_THREE_LINE_UP_POSE
                        : Constants.Arena.BLUE_SAMPLE_THREE_LINE_UP_POSE;

                    driveSubsystem.setTargetPosition(targetPose);

                    armSubsystem.setArmPosition(Constants.ArmPosition.SAMPLE_INTAKE);
                    manipulatorSubsystem.setWristPosition(Constants.Manipulator.SAMPLE_INTAKE_WRIST_POSITION);
                    manipulatorSubsystem.setClawPosition(Constants.Manipulator.CLAW_OPEN_POSITION);
                    manipulatorSubsystem.setRollerPower(Constants.Manipulator.ROLLER_INTAKE_SPEED);
                },
                () -> {
                    driveSubsystem.driveToPosition(Constants.Drive.MAX_LINEAR_SPEED,Constants.Drive.MAX_ANGULAR_SPEED);
                },
                () -> {
                    if (
                        driveSubsystem.getDistanceToPose() < Constants.Drive.MINIMUM_TARGET_LINEAR_ERROR &&
                            driveSubsystem.getAngleToPose() < Constants.Drive.MINIMUM_TARGET_ANGULAR_ERROR &&
                            Math.abs(Constants.ArmPosition.SAMPLE_INTAKE.SHOULDER_POSITION - armSubsystem.getShoulderPosition()) < Constants.Arm.SHOULDER_MINIMUM_ERROR &&
                            Math.abs(Constants.ArmPosition.SAMPLE_INTAKE.LINEAR_SLIDE_POSITION - armSubsystem.getLinearSlidePosition()) < Constants.Arm.LINEAR_SLIDE_MINIMUM_ERROR
                    ) {
                        return true;
                    } else {
                        return false;
                    }
                }
            )
        );

        commandList.add(
            new InitializeRunAndFinishCommand(
                () -> {
                    // Determine target pose based on alliance and sample enum
                    Pose2D targetPose;
                    targetPose = (alliance == Alliance.RED)
                        ? Constants.Arena.RED_SAMPLE_THREE_INTAKE_POSE
                        : Constants.Arena.BLUE_SAMPLE_THREE_INTAKE_POSE;

                    // Set the target position
                    driveSubsystem.setTargetPosition(targetPose);
                },
                () -> {
                    driveSubsystem.driveToPosition(0.25 * Constants.Drive.MAX_LINEAR_SPEED, 0.25 * Constants.Drive.MAX_ANGULAR_SPEED);
                },
                () -> {
                    if (
                        driveSubsystem.getDistanceToPose() < Constants.Drive.MINIMUM_TARGET_LINEAR_ERROR &&
                            driveSubsystem.getAngleToPose() < Constants.Drive.MINIMUM_TARGET_ANGULAR_ERROR
                    ) {
                        return true;
                    } else {
                        return false;
                    }
                }
            )
        );

        commandList.add(
            scoreSampleCommand
        );

        commandList.add(
            idleAfterScoreSampleCommand
        );

        commandList.add(
            new InitializeRunAndFinishCommand(
                () -> {
                    Pose2D targetPose;
                    targetPose = (alliance == Alliance.RED)
                        ? Constants.Arena.RED_READY_CLIMB_POSE
                        : Constants.Arena.BLUE_READY_CLIMB_POSE;

                    driveSubsystem.setTargetPosition(targetPose);
                    armSubsystem.setArmPosition(Constants.ArmPosition.CLIMB);
                    manipulatorSubsystem.setRollerPower(Constants.Manipulator.ROLLER_INTAKE_SPEED);
                    manipulatorSubsystem.setWristPosition(Constants.Manipulator.CLIMB_WRIST_POSITION);
                },
                () -> {
                    driveSubsystem.driveToPosition(Constants.Drive.MAX_LINEAR_SPEED,Constants.Drive.MAX_ANGULAR_SPEED);
                },
                () -> {
                    if (
                        driveSubsystem.getDistanceToPose() < Constants.Drive.MINIMUM_TARGET_LINEAR_ERROR &&
                            driveSubsystem.getAngleToPose() < Constants.Drive.MINIMUM_TARGET_ANGULAR_ERROR
                    ) {
                        return true;
                    } else {
                        return false;
                    }
                }
            )
        );

        commandList.add(
            new InitializeRunAndFinishCommand(
                () -> {
                    Pose2D targetPose;
                    targetPose = (alliance == Alliance.RED)
                        ? Constants.Arena.RED_CLIMB_POSE
                        : Constants.Arena.BLUE_CLIMB_POSE;

                    driveSubsystem.setTargetPosition(targetPose);
                    armSubsystem.setArmPosition(Constants.ArmPosition.CLIMB);
                    manipulatorSubsystem.setRollerPower(Constants.Manipulator.ROLLER_INTAKE_SPEED);
                    manipulatorSubsystem.setWristPosition(Constants.Manipulator.CLIMB_WRIST_POSITION);
                },
                () -> {
                    driveSubsystem.driveToPosition(0.5 * Constants.Drive.MAX_LINEAR_SPEED, 0.5 * Constants.Drive.MAX_ANGULAR_SPEED);
                },
                () -> {
                    return false;
                }
            )
        );

         */

        SequentialCommandGroup autonomousCommand = new SequentialCommandGroup(
            commandList
        );

        autonomousCommand.schedule();

        driveSubsystem.schedule();
        driveSubsystem.setIMU(new Rotation2d(0));
        armSubsystem.schedule();
        manipulatorSubsystem.schedule();
        visionLocalizationSubsystem.schedule();
        visionLocalizationCommand.schedule();

    }

}
