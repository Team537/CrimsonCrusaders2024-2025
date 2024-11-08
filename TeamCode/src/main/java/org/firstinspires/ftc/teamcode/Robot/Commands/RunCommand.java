package org.firstinspires.ftc.teamcode.Robot.Commands;

import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.CommandBase;

public class RunCommand extends CommandBase {

    //the runnable which will run on the commands execution
    Runnable runnable;

    public RunCommand(Runnable runnable) {
        this.runnable = runnable;
    }

    public void execute() {
        runnable.run();
    }

}
