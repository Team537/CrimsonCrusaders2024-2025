package org.firstinspires.ftc.teamcode.Robot.Commands;

import org.firstinspires.ftc.teamcode.Utilities.CommandSystem.CommandBase;

import java.util.LinkedHashMap;
import java.util.function.Supplier;

public class SelectableCommandGroup {

    LinkedHashMap<String,CommandBase> commandList;
    LinkedHashMap<String,Supplier<String>> directiveList;
    String activeCommand;

    public SelectableCommandGroup(LinkedHashMap<String,CommandBase> commandList, LinkedHashMap<String,Supplier<String>> directiveList) {
        this.commandList = commandList;
        this.directiveList = directiveList;
    }

    public void setCommand(String key) {
        if (commandList.containsKey(key)) {
            activeCommand = key;
            commandList.get(activeCommand).initialize();
        }
    }

    public void execute() {
        commandList.get(activeCommand).execute();

        String directive = directiveList.get(activeCommand).get();
        if (directive != "") {

            char directiveIndicator = directive.charAt(0);
            String directiveMessage = directive.substring(1);

            switch (directiveIndicator) {
                case 'g':
                    setCommand(directiveMessage);
                    break;
                case 'r':
                    setCommand(activeCommand);
                    break;
            }

        }

    }



}
