package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmStateCommand;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.enums.ArmState;
import frc.robot.enums.SuperState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CommandFactory {

    public static SequentialCommandGroup commandFactory(
        SuperState desiredState, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem
    ) {
        if (armSubsystem.getState() == ArmState.eStow) {
            return new SequentialCommandGroup(
                new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem),
                new SetArmStateCommand(desiredState.getArmState(), armSubsystem)
            );
        }
        else {
            return new SequentialCommandGroup(
                new SetArmStateCommand(ArmState.eStow, armSubsystem),
                new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem),
                new SetArmStateCommand(desiredState.getArmState(), armSubsystem)
            );
        }
    }
}
