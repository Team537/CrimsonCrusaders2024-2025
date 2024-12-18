package org.firstinspires.ftc.teamcode.Robot.Commands;

import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.CommandBase;

import java.util.LinkedList;

public class SequentialCommandGroup extends CommandBase {

    LinkedList<CommandBase> commandList;
    int index = 0;

    public SequentialCommandGroup(LinkedList<CommandBase> commandList) {
        this.commandList = commandList;
    }

    public void initialize() {
        commandList.get(index).initialize();
    }

    public void execute() {
        commandList.get(index).execute();
        if (commandList.get(index).isFinished()) {
            commandList.get(index).onFinished();
            index++;
            commandList.get(index).initialize();
        }
    }

    public void setIndex(int index) {
        this.index = index;
    }

}
