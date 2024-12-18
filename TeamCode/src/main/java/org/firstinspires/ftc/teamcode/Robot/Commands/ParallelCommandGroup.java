package org.firstinspires.ftc.teamcode.Robot.Commands;

import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.CommandBase;

import java.util.ArrayList;

public class ParallelCommandGroup extends CommandBase {

    ArrayList<CommandBase> commandList;

    public ParallelCommandGroup(ArrayList<CommandBase> commandList) {
        this.commandList = commandList;
    }

    public void initialize() {
        for (CommandBase command : commandList) {
            command.initialize();
        }
    }

    public void execute() {
        for (CommandBase command : commandList) {
            command.execute();
            if (command.isFinished()) {
                command.onFinished();
            }
        }
    }

}
