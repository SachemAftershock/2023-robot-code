package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class OutputCubeCommand extends InstantCommand {

  private IntakeSubsystem mIntakeSubsystem;

  public OutputCubeCommand(IntakeSubsystem intakeSubsystem) {
    mIntakeSubsystem = intakeSubsystem;
    addRequirements(mIntakeSubsystem);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    mIntakeSubsystem.outputCube();
  }
}
