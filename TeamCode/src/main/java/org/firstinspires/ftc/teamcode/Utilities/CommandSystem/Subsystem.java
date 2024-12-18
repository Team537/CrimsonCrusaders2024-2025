package org.firstinspires.ftc.teamcode.Utilities.CommandSystem;

import org.firstinspires.ftc.teamcode.Robot.Commands.RunCommand;

public abstract class Subsystem {

    CommandBase command = new RunCommand(
        this::periodic
    );
    public void periodic() {}

    public void schedule() {
        command.schedule();
    }

    public void cancel() {
        command.cancel();
    }

}
