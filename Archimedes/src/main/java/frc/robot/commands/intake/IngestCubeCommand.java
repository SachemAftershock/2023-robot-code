package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ButtonBoxPublisher;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.subsystems.IntakeSubsystem;

public class IngestCubeCommand extends InstantCommand {

  private IntakeSubsystem mIntakeSubsystem;

  public IngestCubeCommand(IntakeSubsystem intakeSubsystem) {
    mIntakeSubsystem = intakeSubsystem;
    addRequirements(mIntakeSubsystem);
  }

  @Override
  public void execute() {
    System.out.println("Ingesting a cube");
    mIntakeSubsystem.ingestCube();
    ButtonBoxPublisher.enableLed(LedPosition.eIngest);
  }
}
