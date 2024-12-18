package org.firstinspires.ftc.teamcode.Robot.Commands;

import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.CommandBase;

public class OneTimeCommand  extends CommandBase {

    //the runnable which will run on the commands execution
    Runnable initializer;

    public OneTimeCommand(Runnable initializer) {
        this.initializer = initializer;
    }

    public void execute() {
        initializer.run();
    }

    public boolean isFinished() {
        return true;
    }

}
