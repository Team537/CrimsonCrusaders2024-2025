package org.firstinspires.ftc.teamcode.Utilities.CommandSystem;

import java.util.ArrayList;

public class CommandScheduler {

    private static CommandScheduler INSTANCE;

    private CommandScheduler() {}

    public static CommandScheduler getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new CommandScheduler();
        }
        return INSTANCE;
    }

    public void schedule(CommandBase command) {

        if (! (commandList.contains(command) || pendingCommandList.contains(command) ) ) {
            pendingCommandList.add(command);
        }

    }

    public void cancel(CommandBase command) {
        if (commandList.contains(command) && !pendingDeletionCommandList.contains(command)) {
            pendingDeletionCommandList.add(command);
        }
    }

    public void reset() {
        commandList.clear();
        pendingCommandList.clear();
        pendingDeletionCommandList.clear();
    }

    private ArrayList<CommandBase> commandList = new ArrayList<CommandBase>();
    private ArrayList<CommandBase> pendingCommandList = new ArrayList<CommandBase>();
    private ArrayList<CommandBase> pendingDeletionCommandList =  new ArrayList<CommandBase>();

    public void run() {

        //scheduling all pending commands at one time so scheduling while commands are running doesn't cause any issues.
        for (CommandBase pendingCommand : pendingCommandList) {
            //changing the command from the pending list to the command list
            commandList.add(pendingCommand);

            //running the initialize method on the command
            pendingCommand.initialize();
        }
        pendingCommandList.clear();

        //canceling all pending commands at one time so cancelling while commands are running doesn't cause any issues.
        for (CommandBase pendingDeletionCommand : pendingDeletionCommandList) {

            //removing the command from both lists
            commandList.remove(pendingDeletionCommand);

            //running the onFinished method on the command
            pendingDeletionCommand.onFinished();
        }
        pendingDeletionCommandList.clear();

        //running all commands
        for (CommandBase command : commandList) {
            command.execute();
            if (command.isFinished()) {
                command.cancel();
            }
        }
    }

}
