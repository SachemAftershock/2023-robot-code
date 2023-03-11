package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ButtonBoxPublisher;
import frc.robot.commands.arm.SetArmStateCommand;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.enums.ArmState;
import frc.robot.enums.ElevatorState;
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
            case eStow:
                position = LedPosition.eStow;
                break;
            case ePlayerStation:
                position = LedPosition.ePlayerStation;
                break;
            case eLow:
                position = LedPosition.eLow;
                break;
            case eFloor:
                position = LedPosition.eLow;
                break;
            case eMid:
                position = LedPosition.eMid;
                break;
            case eHigh:
                position = LedPosition.eHigh;
                break;
            default:
                position = null;
                break;
        }

        ButtonBoxPublisher.enableLed(position);

        if (desiredState.getArmState() == ArmState.eStowEmpty) {
            System.out.println("-----------RETURNING TO STOW--------------");
            return new SequentialCommandGroup(
                new SetArmStateCommand(ArmState.eStowEmpty, armSubsystem),
                new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem)
            );
        }
        else if (desiredState == SuperState.eFloor) {
            System.out.println("--------FLOOR SEQUENCE-----------");
            return new SequentialCommandGroup(
                new SetElevatorStateCommand(ElevatorState.eRaised, elevatorSubsystem),
                new SetArmStateCommand(ArmState.eLow, armSubsystem),
                new SetElevatorStateCommand(ElevatorState.eStowEmpty, elevatorSubsystem)
            );

        }
        else {
            System.out.println("----------STARTING EXTENSION--------------");
            return new SequentialCommandGroup(
                new SetArmStateCommand(ArmState.eStowEmpty, armSubsystem),
                new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem),
                // new SetArmStateCommand(ArmState.eStowEmpty, armSubsystem),
                new SetArmStateCommand(desiredState.getArmState(), armSubsystem)
            );
        }
    }
}
