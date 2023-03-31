package frc.robot.auto.fieldOrientedTrajectoryAuto;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

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
import frc.robot.commands.CommandFactory;
import frc.robot.commands.arm.SetArmStateCommand;
import frc.robot.commands.drive.DriveToWaypointCommand;
import frc.robot.commands.drive.FollowTrajectoryCommandFactory;
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

public class BalanceDockMobility extends SequentialCommandGroup {

    private final DriveSubsystem mDrive;
    private final ElevatorSubsystem mElevator;
    private final ArmSubsystem mArm;
    private final IntakeSubsystem mIntake;

    TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.kAutoMaxVelocityMetersPerSecond * 0.3, DriveConstants.kMaxAccelerationMetersPerSecondSquared
    );
    PathPlannerTrajectory chargeStation = PathPlanner.loadPath("ChargeStation+Mobility", new PathConstraints(DriveConstants.kAutoMaxVelocityMetersPerSecond * .2, DriveConstants.kMaxAccelerationMetersPerSecondSquared));

    Trajectory pathToCube = TrajectoryGenerator.generateTrajectory(
        new Pose2d(new Translation2d(1.9, 1.08), new Rotation2d(1 / 2 * Math.PI)),
        List.of(new Translation2d(1.9, 1.08), new Translation2d(4.98, 0.92)), new Pose2d(6.45, 2.11, new Rotation2d()),
        config
    );

    Trajectory pathToCommunity = TrajectoryGenerator.generateTrajectory(
        new Pose2d(),
        List.of(new Translation2d(6.46, 2.11), new Translation2d(5.23, 0.74), new Translation2d(3.19, 0.72)),
        new Pose2d(1.9, 1.08, new Rotation2d()), config
    );

    Trajectory pathToChargeStation = TrajectoryGenerator.generateTrajectory(
        new Pose2d(), List.of(new Translation2d(1.9, 1.08), new Translation2d(2.39, 2.37)),
        new Pose2d(3.92, 2.39, new Rotation2d()), config
    );

    public BalanceDockMobility(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm, IntakeSubsystem intake) {

        mDrive = drive;
        mElevator = elevator;
        mArm = arm;
        mIntake = intake;
        addCommands(
            // new InstantCommand(() -> RobotContainer.setIsCube()),
            // CommandFactory.HandleSuperStructureSequence(SuperState.eHigh, mElevator, mArm, mIntake), 
            // new DelayCommand(0.5),
            // new EjectCubeCommand(mIntake),
            // new DelayCommand(0.5),
            // new StopIntakeCommand(mIntake),
            // CommandFactory.HandleSuperStructureSequence(SuperState.eStow, mElevator, mArm, mIntake),
            // //mDrive.followPathTrajectoryBlue(true, examplePath)
            mDrive.followPathTrajectoryXLinear(true, chargeStation)
            //new LinearDriveCommand(mDrive, -3.5, CardinalDirection.eX)
        );
    }

}
