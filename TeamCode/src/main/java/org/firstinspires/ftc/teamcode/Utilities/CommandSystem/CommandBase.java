package org.firstinspires.ftc.teamcode.Utilities.CommandSystem;



public abstract class CommandBase {

    public void schedule() {
        CommandScheduler.getInstance().schedule(this);
    }

    public void cancel() {
        CommandScheduler.getInstance().cancel(this);
    }

    public boolean isFinished() {
        return false;
    }

    public void execute() {}
    public void initialize() {}
    public void onFinished() {}

}
