package org.firstinspires.ftc.teamcode.Robot.Commands;

import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.CommandBase;

public class InitializeAndRunCommand extends CommandBase {

    //the runnables which will run on the commands execution
    Runnable initializer;
    Runnable runnable;

    /**
     * creates the command
     * @param initializer the command to run initially
     * @param runnable the command to run repeatedly upon execution
     */
    public InitializeAndRunCommand(Runnable initializer, Runnable runnable) {
        this.initializer = initializer;
        this.runnable = runnable;
    }

    public void initialize() {
        initializer.run();
    }
    public void execute() {
        runnable.run();
    }

}
