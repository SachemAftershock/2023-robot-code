package frc.robot.auto.fieldOrientedTrajectoryAuto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

public class AutoPathTwoNoCharge extends SequentialCommandGroup{

    private final DriveSubsystem mDrive; 
    private final ElevatorSubsystem mElevator;
    private final ArmSubsystem mArm;
    private final IntakeSubsystem mIntake;
    private Pose2d mStartingPose = new Pose2d(1.9, 4.89, new Rotation2d(Math.PI));
    //private Transform2d x = new Pose2d(1.9, 4.89, new Rotation2d(Math.PI));

    private Transform2d mTransform2d = new Transform2d(new Pose2d(), mStartingPose);
    
    
    TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.kAutoMaxVelocityMetersPerSecond * 0.3,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared
    );

    Trajectory pathToCone = TrajectoryGenerator.generateTrajectory(new Pose2d(new Translation2d(1.9, 4.89), new Rotation2d(Math.PI)),
        List.of(
        new Translation2d(6.67, 4.6)
        ), new Pose2d(7.78, 4.61, new Rotation2d()), config);

    Trajectory pathToCommunity = TrajectoryGenerator.generateTrajectory(new Pose2d(),
        List.of(
        new Translation2d(3.64, 4.76),
        new Translation2d(2.09, 4.38)
        ), new Pose2d(1.9, 3.83, new Rotation2d()), config);

    Trajectory pathToChargeStation = TrajectoryGenerator.generateTrajectory(new Pose2d(),
        List.of(
        new Translation2d(2.15, 3.2)
        ), new Pose2d(3.9, 3.44, new Rotation2d()), config);


    Trajectory mNewPathToCone = pathToCone.transformBy(mTransform2d);

    public AutoPathTwoNoCharge(DriveSubsystem drive, ElevatorSubsystem elevator, ArmSubsystem arm, IntakeSubsystem intake) {

        mDrive = drive;
        mElevator = elevator;
        mArm = arm;
        mIntake = intake;

        //pathToCone.transformBy(mStartingPose);

        addCommands(
            //Places cone preloaded in robot
            // new InstantCommand(() -> RobotContainer.toggleIsCone()),
            CommandFactory.HandleSuperStructureSequence(SuperState.eHigh, mElevator, mArm, mIntake),
            new EjectConeCommand(mIntake),
            new DelayCommand(0.5),
            new StopIntakeCommand(mIntake),
            CommandFactory.HandleSuperStructureSequence(SuperState.eStow, mElevator, mArm, mIntake),
            
            //Robot moves to cone on field
            FollowTrajectoryCommandFactory.generateCommand(mDrive, pathToCone),
            new RotateDriveCommand(mDrive, 180),
            
            //Sequence for picking up cone and stowing
            new InstantCommand(() -> RobotContainer.setIsCone()),
            new IngestConeCommand(mIntake),
            CommandFactory.HandleSuperStructureSequence(SuperState.eLow, mElevator, mArm, mIntake),
            new DelayCommand(0.5),
            new StopIntakeCommand(mIntake),
            CommandFactory.HandleSuperStructureSequence(SuperState.eStow, mElevator, mArm, mIntake)

            //Driving back
            // new RotateDriveCommand(mDrive, 180),
            // FollowTrajectoryCommandFactory.generateCommand(mDrive, pathToCommunity),
            // new DriveToWaypointCommand(SlotState.ePosition1.getPosition(), mDrive),

            // //Placing cone sequence
            // CommandFactory.HandleSuperStructureSequence(SuperState.eHigh, mElevator, mArm, mIntake),
            // new EjectConeCommand(mIntake),
            // new DelayCommand(0.5),
            // new StopIntakeCommand(mIntake),
            // CommandFactory.HandleSuperStructureSequence(SuperState.eStow, mElevator, mArm, mIntake)
        );
    }



    
}
