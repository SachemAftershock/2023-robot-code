package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ButtonBoxPublisher;
import frc.robot.commands.arm.SetArmStateCommand;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.enums.ArmState;
import frc.robot.enums.SuperState;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CommandFactory {

    public static SequentialCommandGroup HandleSuperStructureSequence(
        SuperState desiredState, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem
    ) {
        LedPosition position;
        switch (desiredState) {
            case eHigh:
                position = LedPosition.eHigh;
                break;
            case eMid:
                position = LedPosition.eMid;
                break;
            case eLow:
                position = LedPosition.eLow;
                break;
            case ePlayerStation:
                position = LedPosition.ePlayerStation;
                break;
            case eStow:
                position = LedPosition.eStow;
                break;
            default:
                position = null;
        }

        ButtonBoxPublisher.enableLed(position);

        if (armSubsystem.getState() == ArmState.eStowEmpty) {
            System.out.println("Starting sequence");
            return new SequentialCommandGroup(
                new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem),
                new SetArmStateCommand(desiredState.getArmState(), armSubsystem)
            );
        }
        else {
            System.out.println("This should retract arm first");
            return new SequentialCommandGroup(
                new SetArmStateCommand(ArmState.eStowEmpty, armSubsystem),
                //new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem),
                new SetArmStateCommand(desiredState.getArmState(), armSubsystem)
            );
        }
    }
}
