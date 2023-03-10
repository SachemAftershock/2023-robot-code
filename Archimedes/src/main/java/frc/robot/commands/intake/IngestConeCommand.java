package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ButtonBoxPublisher;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.subsystems.IntakeSubsystem;

public class IngestConeCommand extends InstantCommand {
  private IntakeSubsystem mIntakeSubsystem;

  public IngestConeCommand(IntakeSubsystem intakeSubsystem) {
    mIntakeSubsystem = intakeSubsystem;
    addRequirements(mIntakeSubsystem);
  }

  @Override
  public void execute() {
    mIntakeSubsystem.ingestCone();
    ButtonBoxPublisher.enableLed(LedPosition.eIngest);
  }

}