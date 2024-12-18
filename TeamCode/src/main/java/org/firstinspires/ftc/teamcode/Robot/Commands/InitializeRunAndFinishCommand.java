package org.firstinspires.ftc.teamcode.Robot.Commands;

import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.CommandBase;

import java.util.function.Supplier;

public class InitializeRunAndFinishCommand extends CommandBase {
    //the runnables which will run on the commands execution
    Runnable initializer;
    Runnable runnable;
    Supplier<Boolean> isFinished;

    /**
     * creates the command
     * @param initializer the command to run initially
     * @param runnable the command to run repeatedly upon execution
     * @param isFinished the test for is finished
     */
    public InitializeRunAndFinishCommand(Runnable initializer, Runnable runnable, Supplier<Boolean> isFinished) {
        this.initializer = initializer;
        this.runnable = runnable;
        this.isFinished = isFinished;
    }

    public void initialize() {
        initializer.run();
    }
    public void execute() {
        runnable.run();
    }
    public boolean isFinished() {
        return isFinished.get();
    }

}
