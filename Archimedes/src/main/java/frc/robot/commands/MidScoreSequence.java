package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.enums.ElevatorState;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.commands.intake.SmartIngestCommand;
import frc.robot.enums.ElevatorState;
import frc.robot.enums.ArmState;
import frc.robot.enums.ElevatorState;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.ButtonBoxPublisher;
import frc.robot.auto.DelayCommand;
import frc.robot.commands.arm.SetArmStateCommand;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class MidScoreSequence extends SequentialCommandGroup{

    public MidScoreSequence(ArmSubsystem arm, ElevatorSubsystem elevator, IntakeSubsystem intake) {
        addCommands(
            new InstantCommand(() -> ButtonBoxPublisher.enableLed(LedPosition.eHumanPlayerRight)),
            new SetElevatorStateCommand(ElevatorState.eMid, elevator),
            new SetArmStateCommand(ArmState.eMid, arm)

        );
        
    }
    
}
