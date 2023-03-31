package frc.robot.auto.fieldOrientedTrajectoryAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.auto.DelayCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.intake.EjectConeCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.enums.SuperState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ConeHigh extends SequentialCommandGroup {

    private final ElevatorSubsystem mElevator;
    private final ArmSubsystem mArm;
    private final IntakeSubsystem mIntake;

    public ConeHigh(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm, IntakeSubsystem intake) {
        mElevator = elevator;
        mArm = arm;
        mIntake = intake;

        addCommands(
            // Places cone preloaded in robot
            new InstantCommand(() -> RobotContainer.setIsCone()),
            CommandFactory.HandleSuperStructureSequence(SuperState.eHigh, mElevator, mArm, mIntake),
            new EjectConeCommand(mIntake), 
            new DelayCommand(0.5), 
            new StopIntakeCommand(mIntake),
            CommandFactory.HandleSuperStructureSequence(SuperState.eStow, mElevator, mArm, mIntake)

        );
    }

}
