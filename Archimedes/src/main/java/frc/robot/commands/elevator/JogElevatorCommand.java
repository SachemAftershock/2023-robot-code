package frc.robot.commands.elevator;

import frc.robot.enums.ElevatorState;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class JogElevatorCommand extends CommandBase {
  private ElevatorState mDesiredState;
  private ElevatorSubsystem mElevatorSubsystem;
  private double mJogValue;

  public JogElevatorCommand(ElevatorSubsystem elevatorSubsystem, double jogValue) {
    mElevatorSubsystem = elevatorSubsystem;
    mJogValue = jogValue;
    addRequirements(mElevatorSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    System.out.println("Jog elevator");
    mElevatorSubsystem.jogSetpoint(mJogValue);
  }

  @Override
  public void end(boolean interrupted) {
  }
}

