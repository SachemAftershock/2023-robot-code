// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.SubsystemManager;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.Constants.LoadingZone;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.drive.DriveToCoordinateCommand;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.ManualDriveCommand;
import frc.robot.commands.drive.SetWaypointCommand;
import frc.robot.commands.intake.IngestConeCommand;
import frc.robot.commands.intake.IngestCubeCommand;
import frc.robot.commands.intake.OutputConeCommand;
import frc.robot.commands.intake.OutputCubeCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.enums.SuperState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.FieldConstants.kPlacingPoses;

import java.util.function.Function;

import static frc.robot.Constants.DriveConstants.kRotationScalingConstant;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private static boolean mIsCone = true;

    private final ElevatorSubsystem mElevatorSubsystem = ElevatorSubsystem.getInstance();
    private final IntakeSubsystem mIntakeSubsystem = IntakeSubsystem.getInstance();
    private final DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
    private final ArmSubsystem mArmSubsystem = ArmSubsystem.getInstance();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(mElevatorSubsystem, mIntakeSubsystem);

    private final CommandJoystick mPrimaryThrottleController = new CommandJoystick(ControllerConstants.kPrimaryThrottleControllerPort);
    private final CommandJoystick mPrimaryTwistController = new CommandJoystick(ControllerConstants.kPrimaryTwistControllerPort);
    private final ButtonBox mButtonBox = new ButtonBox(ControllerConstants.kButtonBoxPort);


    private DriveToCoordinateCommand mDriveToCoordinateCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        mDriveSubsystem.setDefaultCommand(
            new ManualDriveCommand(
                mDriveSubsystem, mArmSubsystem::getState,
                () -> -modifyAxis(mPrimaryThrottleController.getX()) * DriveConstants.kMaxVelocityMetersPerSecond,
                () -> -modifyAxis(mPrimaryThrottleController.getY()) * DriveConstants.kMaxVelocityMetersPerSecond,
                () -> -modifyAxis(mPrimaryTwistController.getTwist()) * DriveConstants.kMaxAngularVelocityRadiansPerSecond * kRotationScalingConstant
            )
        );
    }

    public void initializeSubsystems() {
        mSubsystemManager.initialize();
    }

    private void configureButtonBindings() {
        mPrimaryThrottleController.getTrigger().onTrue(new InstantCommand(() -> {
            mDriveToCoordinateCommand = new DriveToCoordinateCommand(mDriveSubsystem.getWaypoint(), mDriveSubsystem);
            mDriveToCoordinateCommand.schedule();
        })).onFalse(new InstantCommand(() -> {
            mDriveToCoordinateCommand.cancel();
            ButtonBoxPublisher.enableLed(mDriveSubsystem.getLedPosition());
        }));



        mButtonBox.cubeToggle().onTrue(new InstantCommand(() -> RobotContainer.toggleIsCone()));
        mButtonBox.coneToggle().onTrue(new InstantCommand(() -> RobotContainer.toggleIsCone()));

        mButtonBox.ingestIntake()
            .onTrue(RobotContainer.isCone() ? new IngestConeCommand(mIntakeSubsystem) : new IngestCubeCommand(mIntakeSubsystem))
            .onFalse(new InstantCommand(() -> {
                (new StopIntakeCommand(mIntakeSubsystem)).schedule();
                ButtonBoxPublisher.disableLed(LedPosition.eIngest);
            }));

        mButtonBox.ejectIntake()
            .onTrue(RobotContainer.isCone() ? new OutputConeCommand(mIntakeSubsystem) : new OutputCubeCommand(mIntakeSubsystem))
            .onFalse(new InstantCommand(() -> {
                (new StopIntakeCommand(mIntakeSubsystem)).schedule();
                ButtonBoxPublisher.disableLed(LedPosition.eEject);
            }));

        mButtonBox.highPosition().onTrue(CommandFactory.commandFactory(SuperState.eHigh, mElevatorSubsystem, mArmSubsystem));
        mButtonBox.mediumPosition().onTrue(CommandFactory.commandFactory(SuperState.eMid, mElevatorSubsystem, mArmSubsystem));
        mButtonBox.floorPosition().onTrue(CommandFactory.commandFactory(SuperState.eLow, mElevatorSubsystem, mArmSubsystem));

        mButtonBox.humanPlayerPostion().onTrue(CommandFactory.commandFactory(SuperState.ePlayerStation, mElevatorSubsystem, mArmSubsystem));
        mButtonBox.stowPostion().onTrue(CommandFactory.commandFactory(SuperState.eStow, mElevatorSubsystem, mArmSubsystem));

        mButtonBox.cancel().onTrue(new InstantCommand(() -> {
            CommandScheduler.getInstance().cancelAll();
            ButtonBoxPublisher.enableLed(LedPosition.eCancel);
        }));

        mButtonBox.cone1().onTrue(new SetWaypointCommand(kPlacingPoses[0].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo1));
        mButtonBox.cube2().onTrue(new SetWaypointCommand(kPlacingPoses[1].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo2));
        mButtonBox.cone3().onTrue(new SetWaypointCommand(kPlacingPoses[2].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo3));

        mButtonBox.cone4().onTrue(new SetWaypointCommand(kPlacingPoses[3].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo4));
        mButtonBox.cube5().onTrue(new SetWaypointCommand(kPlacingPoses[4].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo5));
        mButtonBox.cone6().onTrue(new SetWaypointCommand(kPlacingPoses[5].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo6));

        mButtonBox.cone7().onTrue(new SetWaypointCommand(kPlacingPoses[6].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo7));
        mButtonBox.cube8().onTrue(new SetWaypointCommand(kPlacingPoses[7].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo8));
        mButtonBox.cone9().onTrue(new SetWaypointCommand(kPlacingPoses[8].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo9));

        // TODO: Figure out stuff w human player coord left/right position
        mButtonBox.leftHumanStation().onTrue(new SetWaypointCommand(LoadingZone.kDoubleSubstationPose, mDriveSubsystem, LedPosition.eHumanPlayerLeft));
        // mButtonBox.rightHumanStation().onTrue(new SetWaypointCommand(,
        // mDriveSubsystem));
        Function<Boolean, InstantCommand> jogElevatorCommand = (isUp) -> new InstantCommand(() -> {
            if (mButtonBox.isJoystickEnabled()) mElevatorSubsystem.jogElevator(true);
        });

        Function<Boolean, InstantCommand> jogArmCommand = (isOut) -> new InstantCommand(() -> {
            if (mButtonBox.isJoystickEnabled()) mArmSubsystem.jogArm(isOut);
        });

        mButtonBox.upJoystickButton().onTrue(jogElevatorCommand.apply(true)).onFalse(new InstantCommand(() -> mElevatorSubsystem.stop()));
        mButtonBox.downJoystickButton().onTrue(jogElevatorCommand.apply(false)).onFalse(new InstantCommand(() -> mElevatorSubsystem.stop()));
        mButtonBox.rightJoystickButton().onTrue(jogArmCommand.apply(true)).onFalse(new InstantCommand(() -> mArmSubsystem.stop()));
        mButtonBox.leftJoystickButton().onTrue(jogArmCommand.apply(false)).onFalse(new InstantCommand(() -> mArmSubsystem.stop()));

        mButtonBox.enableJoystick().onTrue(new InstantCommand(() -> mButtonBox.toggleJoystick()));
    }

    public static void toggleIsCone() {
        mIsCone = !mIsCone;

        ButtonBoxPublisher.enableLed(mIsCone ? LedPosition.eConeActive : LedPosition.eCubeActive);
    }

    public static boolean isCone() {
        return mIsCone;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // TrajectoryConfig config = new TrajectoryConfig(
        // DriveConstants.kMaxVelocityMetersPerSecond * 0.3,
        // DriveConstants.kMaxAccelerationMetersPerSecondSquared
        // );

        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(),
        // List.of(new Translation2d(2.0, -0.5)
        // // new Translation2d(0, 1.2)//,
        // // new Translation2d(1, 1.5)
        // // new Translation2d(2.2,0)
        // ), new Pose2d(1.0, -0.4, new Rotation2d()), config);

        // Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
        // new Pose2d(), List.of(new Translation2d(0.5, 1.5), new Translation2d(1.0,
        // 1.5), new Translation2d(1.5, 1.5)),
        // new Pose2d(1.5, 1.5, new Rotation2d()), config
        // );

        return new LinearDriveCommand(mDriveSubsystem, 2.0, CardinalDirection.eX);

        // return new SequentialCommandGroup(
        // //FollowTrajectoryCommandFactory.generateCommand(mDriveSubsystem,
        // trajectory),
        // //new RotateDriveCommand(mDriveSubsystem, 90),
        // //FollowTrajectoryCommandFactory.generateCommand(mDriveSubsystem,
        // trajectory2)
        // //new RotateDriveCommand(mDriveSubsystem, -30)

        // );
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            }
            else {
                return (value + deadband) / (1.0 - deadband);
            }
        }
        else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, DriveConstants.kDriveControllerDeadband);

        // Square the axis
        if (DriveConstants.kSquareAxis) {
            value = Math.copySign(value * value, value);
        }

        return value;
    }
}
