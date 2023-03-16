package frc.robot.commands.intake;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ButtonBoxPublisher;
import frc.robot.RobotContainer;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.subsystems.IntakeSubsystem;

public class SmartIntakeCommand extends InstantCommand {

  private IntakeSubsystem mIntakeSubsystem;

  public SmartIntakeCommand(IntakeSubsystem intakeSubsystem) {
    mIntakeSubsystem = intakeSubsystem;
    addRequirements(mIntakeSubsystem);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    if (RobotContainer.isCone()) {
        mIntakeSubsystem.ingestCone();
    } else {
        mIntakeSubsystem.ingestCube();
    }
    
  }
}
