package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class OutputConeCommand extends InstantCommand {
  
  private IntakeSubsystem mIntakeSubsystem;

  public OutputConeCommand(IntakeSubsystem intakeSubsystem) {
    mIntakeSubsystem = intakeSubsystem;
    addRequirements(mIntakeSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    mIntakeSubsystem.outputCone();
  }
}
