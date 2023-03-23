package frc.robot.commands.elevator;

import frc.robot.enums.ElevatorState;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetElevatorStateCommand extends CommandBase {
  private ElevatorState mDesiredState;
  private ElevatorSubsystem mElevatorSubsystem;

  public SetElevatorStateCommand(ElevatorState desiredState, ElevatorSubsystem elevatorSubsystem) {
    mElevatorSubsystem = elevatorSubsystem;
    mDesiredState = desiredState;
    addRequirements(mElevatorSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("Elevator set to " + mDesiredState);
    mElevatorSubsystem.setDesiredState(mDesiredState); // if broken move to execute
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    mElevatorSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return mElevatorSubsystem.getState() == mDesiredState;
  }
}
