package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ButtonBoxPublisher;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.SetArmStateCommand;
import frc.robot.commands.elevator.SetElevatorLedCommand;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.commands.intake.SmartIngestCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.enums.ArmState;
import frc.robot.enums.ElevatorState;
import frc.robot.enums.SuperState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CommandFactory {

    private static boolean mWasInFloorPosition = false;

    public static SequentialCommandGroup HandleSuperStructureSequence(
        SuperState desiredState, ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem mIntakeSubsystem
    ) {

        // return new SequentialCommandGroup(
        // new SetElevatorLedCommand(desiredState),
        // new SetElevatorStateCommand(desiredState.getElevatorState(),
        // elevatorSubsystem)
        // );

        // }

        if (desiredState.getArmState() == ArmState.eStowEmpty) {
            System.out.println("-----------RETURNING TO STOW--------------");
            return new SequentialCommandGroup(
                new StopIntakeCommand(mIntakeSubsystem),
                new SetElevatorLedCommand(desiredState), 
                new SetArmStateCommand(ArmState.eStowEmpty, armSubsystem),
                new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem)
            );
        }

        else if (desiredState == SuperState.eStow && mWasInFloorPosition) {
            mWasInFloorPosition = false;
            System.out.println("-------------Starting FLOOR retract----------");
            return new SequentialCommandGroup(
                new StopIntakeCommand(mIntakeSubsystem),
                new SetElevatorLedCommand(desiredState), 
                new SetElevatorStateCommand(ElevatorState.eLow, elevatorSubsystem),
                new SetArmStateCommand(ArmState.eStowEmpty, armSubsystem),
                new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem)
            );
        }
        else {
            System.out.println("----------STARTING EXTENSION--------------");
            return new SequentialCommandGroup(
                new SetElevatorLedCommand(desiredState), new SetArmStateCommand(ArmState.eStowEmpty, armSubsystem),
                new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem),
                // new SetArmStateCommand(ArmState.eStowEmpty, armSubsystem),
                new SetArmStateCommand(desiredState.getArmState(), armSubsystem)
            );
        }
    }
}
