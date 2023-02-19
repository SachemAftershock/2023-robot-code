package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeCommand extends InstantCommand {
  
  private IntakeSubsystem mIntakeSubsystem;
  
  public StopIntakeCommand(IntakeSubsystem intakeSubsystem) {
    mIntakeSubsystem = intakeSubsystem;
    addRequirements(mIntakeSubsystem);
  }

  @Override
  public void execute() {
    mIntakeSubsystem.stop();
  }
}
