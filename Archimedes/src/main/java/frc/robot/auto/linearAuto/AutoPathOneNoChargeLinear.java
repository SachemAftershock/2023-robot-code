package frc.robot.auto.linearAuto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.arm.SetArmStateCommand;
import frc.robot.commands.drive.DriveToWaypointCommand;
import frc.robot.commands.drive.FollowTrajectoryCommandFactory;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.commands.intake.EjectConeCommand;
import frc.robot.commands.intake.IngestConeCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.enums.ArmState;
import frc.robot.enums.ElevatorState;
import frc.robot.enums.SlotState;
import frc.robot.enums.SuperState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.auto.DelayCommand;


public class AutoPathOneNoChargeLinear extends SequentialCommandGroup{

    private final DriveSubsystem mDrive; 
    private final ElevatorSubsystem mElevator;
    private final ArmSubsystem mArm;
    private final IntakeSubsystem mIntake;

    public AutoPathOneNoChargeLinear(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm, IntakeSubsystem intake) {

        mDrive = drive;
        mElevator = elevator;
        mArm = arm;
        mIntake = intake;

        addCommands(
            //Places cone preloaded in robot
            new InstantCommand(() -> RobotContainer.setIsCone()),
            CommandFactory.HandleSuperStructureSequence(SuperState.eHigh, mElevator, mArm, mIntake),
            new EjectConeCommand(mIntake),
            new DelayCommand(0.5),
            new StopIntakeCommand(mIntake),
            CommandFactory.HandleSuperStructureSequence(SuperState.eStow, mElevator, mArm, mIntake),
            
            //Robot moves to cone on field
            new LinearDriveCommand(mDrive, 2.0, CardinalDirection.eX),
            new RotateDriveCommand(mDrive, 180),
            
            // //Sequence for picking up cone and stowing
            new IngestConeCommand(mIntake),
            CommandFactory.HandleSuperStructureSequence(SuperState.eFloor, mElevator, mArm, mIntake),
            new DelayCommand(0.5),
            new StopIntakeCommand(mIntake),
            CommandFactory.HandleSuperStructureSequence(SuperState.eStow, mElevator, mArm, mIntake)

            // //Driving back
            // new RotateDriveCommand(mDrive, 180),
            // FollowTrajectoryCommandFactory.generateCommand(mDrive, pathToCommunity),
            // new DriveToWaypointCommand(SlotState.ePosition1.getPosition(), mDrive),

            // //Placing cone sequence
            // CommandFactory.HandleSuperStructureSequence(SuperState.eHigh, mElevator, mArm, mIntake),
            // new EjectConeCommand(mIntake),
            // new DelayCommand(0.5),
            // new StopIntakeCommand(mIntake),
            // CommandFactory.HandleSuperStructureSequence(SuperState.eStow, mElevator, mArm, mIntake),
            // FollowTrajectoryCommandFactory.generateCommand(mDrive, pathToChargeStation)
        );
    }



    
}

