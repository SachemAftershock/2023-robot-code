package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class PrintToConsoleCommand extends InstantCommand {

    private String mMessage;

    public PrintToConsoleCommand(String message) {
        mMessage = message;
    }

    @Override
    public void execute() {
        System.out.println(mMessage);
    }

}
