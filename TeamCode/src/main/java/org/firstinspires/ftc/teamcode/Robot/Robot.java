package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.CommandScheduler;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

//Class to be created by all opModes
public class Robot {

    //This class is a singleton; we must define it as a static object within itself and create a blank constructor
    private static Robot INSTANCE;
    private Robot () {}

    //When this method is called, it will either return the singleton or create a new one if one has not already been created
    public static Robot getInstance()
    {
        if (INSTANCE == null) {
            INSTANCE = new Robot();
        }
        return INSTANCE;
    }

    //defining variables to store the name and type of opMode
    public LinearOpMode opMode;
    private Constants.OpModes opModeType;
    public RobotContainer robotContainer;

    //when this method is run, it will continue until the robot is stopped
    public void run(LinearOpMode opMode, Constants.OpModes opModeType)
    {

        //define these variables and save them
        this.opMode = opMode;
        this.opModeType = opModeType;

        //create a robot container to build methods
        robotContainer = new RobotContainer(opMode,opModeType);

        //runs the init
        robotInit();

        //run the periodic
        while (!opMode.isStarted()) {
            robotPeriodic();
            opMode.telemetry.addLine("init");
            robotInitPeriodic();
        }

        //runs the active init
        robotActiveInit();

        //run the periodic
        while (opMode.opModeIsActive()) {
            robotPeriodic();
            opMode.telemetry.addLine("active");
            robotActivePeriodic();
        }

        robotOnFinished();

    }

    /**
     * Runs when the robot first activates
     */
    private void robotInit() {
        CommandScheduler.getInstance().reset();
    }

    /**
     * Runs periodically while the robot is active or initializing
     */
    private void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.settings();
        opMode.telemetry.update();
    }

    /**
     * Runs periodically while the robot is initializing
     */
    private void robotInitPeriodic() {

    }

    /**
     * Runs initially while the robot is active
     */
    private void robotActiveInit() {
        switch (opModeType) {
            case TELE_OP:
                robotContainer.scheduleTeleOpCommands();
                break;
            case AUTO:
                robotContainer.scheduleAutonomousCommands();
                break;
            case PRE_MATCH_INIT:
                robotContainer.preMatchInit();
                break;
        }

    }

    /**
     * Runs periodically while the robot is active
     */
    private void robotActivePeriodic() {

    }

    /**
     * Runs when the robot finishes running
     */
    private void robotOnFinished() {

    }


}
