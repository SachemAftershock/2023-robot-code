// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;

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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.SubsystemManager;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.ErrorTracker.ErrorType;
import frc.robot.auto.cubeAutoPaths.CubeHighOnly;
import frc.robot.auto.cubeAutoPaths.CubeLowOnly;
import frc.robot.auto.cubeAutoPaths.CubeLowThenChargeStartingMiddle;
import frc.robot.auto.cubeAutoPaths.CubeMidOnly;
import frc.robot.auto.fieldOrientedTrajectoryAuto.*;
import frc.robot.auto.linearAuto.AutoPathConeLinear;
import frc.robot.auto.linearAuto.AutoPathCubeLinear;
import frc.robot.auto.linearAuto.AutoPathTwoNoChargeLinear;
import frc.robot.auto.robotOrientedTrajectoryAuto.AutoPath2NC;
import frc.robot.Constants.LoadingZone;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.FloorPickupSequence;
import frc.robot.commands.FloorRetractSequence;
import frc.robot.commands.LowScoreSequence;
import frc.robot.commands.MidScoreSequence;
import frc.robot.commands.PrintToConsoleCommand;
import frc.robot.commands.arm.AttachHookCommand;
import frc.robot.commands.arm.DetachHookCommand;
import frc.robot.commands.arm.SetArmStateCommand;
import frc.robot.commands.drive.BalanceRobotCommand;
import frc.robot.commands.drive.BalanceRobotContinousCommand;
import frc.robot.commands.drive.BalanceRobotForceDampCommand;
import frc.robot.commands.drive.BalanceRobotTimeIntervalCommand;
import frc.robot.commands.drive.DriveToWaypointCommand;
import frc.robot.commands.drive.FollowTrajectoryCommandFactory;
import frc.robot.commands.drive.HoldPositionCommand;
import frc.robot.commands.drive.LinearDriveCommand;
import frc.robot.commands.drive.ManualDriveCommand;
import frc.robot.commands.drive.RotateDriveCommand;
import frc.robot.commands.drive.SetWaypointCommand;
import frc.robot.commands.elevator.JogElevatorCommand;
import frc.robot.commands.elevator.SetElevatorLedCommand;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.commands.intake.IngestConeCommand;
import frc.robot.commands.intake.IngestCubeCommand;
import frc.robot.commands.intake.EjectConeCommand;
import frc.robot.commands.intake.EjectCubeCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.commands.superstructure.FloorPickupCommand;
import frc.robot.enums.ButtonBoxLedInfo.ButtonPosition;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.enums.ArmState;
import frc.robot.enums.ControllState;
import frc.robot.enums.ElevatorState;
import frc.robot.enums.SuperState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.FieldConstants.kPlacingPoses;
import static frc.robot.Constants.ButtonBoxConstants.kButtonBoxButtonMap;

import java.util.List;
import java.util.ResourceBundle.Control;
import java.util.function.Function;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import static frc.robot.Constants.DriveConstants.kRotationScalingConstant;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private static RobotContainer mInstance;

    private static boolean mIsCone = true;
    private static boolean mWasInFloorPosition = false;
    private static ControllState mControllState = ControllState.eAutomaticControl;

    private final ElevatorSubsystem mElevatorSubsystem = ElevatorSubsystem.getInstance();
    private final IntakeSubsystem mIntakeSubsystem = IntakeSubsystem.getInstance();
    private final DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
    private final ArmSubsystem mArmSubsystem = ArmSubsystem.getInstance();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
        mElevatorSubsystem, mIntakeSubsystem, mArmSubsystem, mDriveSubsystem
    );

    private final CommandJoystick mPrimaryThrottleController = new CommandJoystick(
        ControllerConstants.kPrimaryThrottleControllerPort
    );
    private final CommandJoystick mPrimaryTwistController = new CommandJoystick(
        ControllerConstants.kPrimaryTwistControllerPort
    );
    private final ButtonBox mButtonBox = new ButtonBox(ControllerConstants.kButtonBoxPort);
    private final XboxController mTestController = new XboxController(3);

    private DriveToWaypointCommand mDriveToCoordinateCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        mDriveSubsystem.setDefaultCommand(
            new ManualDriveCommand(
                mDriveSubsystem, mArmSubsystem::getState, mArmSubsystem::isArmStowedEnough,
                mElevatorSubsystem::getElevatorHeight,
                () -> modifyAxis(mPrimaryThrottleController.getY()) * DriveConstants.kManualMaxVelocityMetersPerSecond,
                () -> modifyAxis(mPrimaryThrottleController.getX()) * DriveConstants.kManualMaxVelocityMetersPerSecond,
                () -> -modifyAxis(mPrimaryTwistController.getTwist())
                    * DriveConstants.kMaxAngularVelocityRadiansPerSecond * kRotationScalingConstant,
                mPrimaryThrottleController::getTriggerPressed
            )
        );
    }

    public void initialize() {
        mSubsystemManager.initialize();
        // mArmSubsystem.outputTelemetry();
        // mElevatorSubsystem.outputTelemetry();
        // mIntakeSubsystem.outputTelemetry();
    }

    public void testPeriodic() {
        mElevatorSubsystem.periodic();
    }

    public void periodic() {

        mArmSubsystem.outputTelemetry();
        mElevatorSubsystem.outputTelemetry();
        mIntakeSubsystem.outputTelemetry();
        ErrorTracker.getInstance().reportErrors();

        // System.out.println("CONE----> " + RobotContainer.isCone());

        // if (mTestController.getAButton()) {
        // setBackUpControllState();
        // }

        if (mButtonBox.povUp().getAsBoolean()) {
            // System.out.println("Going up");
            setManualControllState();
            mElevatorSubsystem.jogElevatorUp();
        }
        else if (mButtonBox.povDown().getAsBoolean()) {
            // System.out.println("Going down");
            setManualControllState();
            mElevatorSubsystem.jogElevatorDown();
        }
        else {
            // System.out.println("Stopping");
            // mElevatorSubsystem.stop();
            setAutomaticControllState();
        }

        // if (mButtonBox.povDownLeft().getAsBoolean()) {
        //     System.out.println("Arm going in");
        //     setManualControllState();
        //     mArmSubsystem.jogArmIn();
        // } else if (mButtonBox.povDownRight().getAsBoolean()) {
        //     System.out.println("Arm going out");
        //     setManualControllState();
        //     mArmSubsystem.jogArmOut();
        // } else {
        //     setAutomaticControllState();
        // }

        if(mTestController.getAButton() && Math.abs(mTestController.getLeftY()) > 0.05) {
            setManualControllState();
            mElevatorSubsystem.setManualSpeed(mTestController.getLeftY());
            System.out.println("manual elevator height");
        }

        if(mTestController.getAButton() && Math.abs(mTestController.getLeftX()) > 0.05) {
            
        }

        // if(mTestController.getAButton() && Math.abs(mTestController.getLeftY()) > 0.05) {
        //     mElevatorSubsystem.setManualSpeed(mTestController.getLeftY());
        // }

    }

    public void test() {

        // if (mTestController.getAButton() && (mTestController.getLeftY() > 0.05 || mTestController.getLeftY() < -0.05)) {
        //     //mElevatorSubsystem.setTestSpeed(mTestController.getLeftY() * 0.8);
        //     mElevatorSubsystem.jogElevatorUp();
        // }
        // else {
        //     mElevatorSubsystem.setTestSpeed(0.0);
        // }
        

        if(mTestController.getAButton()) {
            mElevatorSubsystem.jogElevatorUp();
            System.out.println("going up");
        } else if(mTestController.getYButton()) {
            mElevatorSubsystem.jogElevatorDown();
            System.out.println("going down");
        } else {
            mElevatorSubsystem.stop();
        }

        if (mTestController.getAButton() && (mTestController.getLeftX() > 0.05 || mTestController.getLeftX() < -0.05)) {
            mArmSubsystem.setTestSpeed(mTestController.getLeftX() * 0.8);
        }
        else {
            mArmSubsystem.setTestSpeed(0.0);
        }

        // if (mTestController.getRightBumper()) {
        // mArmSubsystem.setHookSpeed(0.2);
        // } else if (mTestController.getLeftBumper()) {
        // mArmSubsystem.setHookSpeed(-0.2);
        // } else {
        // mArmSubsystem.stopHook();
        // }
    }

    public void initializeSubsystems() {
        mSubsystemManager.initialize();
    }

    private void configureButtonBindings() { 
                // mPrimaryThrottleController.getTrigger().onTrue(new InstantCommand(() -> {
        // mDriveToCoordinateCommand = new
        // DriveToWaypointCommand(mDriveSubsystem.getWaypoint(), mDriveSubsystem);
        // mDriveToCoordinateCommand.schedule();
        // })).onFalse(new InstantCommand(() -> {
        // mDriveToCoordinateCommand.cancel();
        // ButtonBoxPublisher.enableLed(mDriveSubsystem.getLedPosition());
        // }));

        // mPrimaryTwistController.getTrigger().whileTrue(new
        // BalanceRobotContinousCommand(mDriveSubsystem));
        mPrimaryTwistController.getTrigger().toggleOnTrue(new HoldPositionCommand(mDriveSubsystem));

        mButtonBox.cubeToggle().onTrue(new InstantCommand(() -> RobotContainer.setIsCube()));
        mButtonBox.coneToggle().onTrue(new InstantCommand(() -> RobotContainer.setIsCone()));

        mButtonBox.ingestIntake().onTrue(new InstantCommand(() -> {
            if (RobotContainer.isCone()) {
                mIntakeSubsystem.ingestCone();
            }
            else {
                mIntakeSubsystem.ingestCube();
            }
        })).onFalse(new InstantCommand(() -> {
            mIntakeSubsystem.stop();
            ButtonBoxPublisher.disableLed(LedPosition.eIngest);
        }));

        mButtonBox.ejectIntake().onTrue(new InstantCommand(() -> {
            if (RobotContainer.isCone()) {
                mIntakeSubsystem.outputCone();
            }
            else {
                mIntakeSubsystem.outputCube();
            }
        })).onFalse(new InstantCommand(() -> {
            mIntakeSubsystem.stop();
            ButtonBoxPublisher.disableLed(LedPosition.eEject);
        }));

        // mButtonBox.highPosition().onTrue(
        //     CommandFactory
        //         .HandleSuperStructureSequence(SuperState.eHigh, mElevatorSubsystem, mArmSubsystem, mIntakeSubsystem)
        // );

        mButtonBox.mediumPosition().onTrue(
            // CommandFactory
            //     .HandleSuperStructureSequence(SuperState.eMid, mElevatorSubsystem, mArmSubsystem, mIntakeSubsystem)
            new SequentialCommandGroup(
                new MidScoreSequence(mArmSubsystem, mElevatorSubsystem, mIntakeSubsystem)

            )
        );

        mButtonBox.floorPosition().onTrue(
            new SequentialCommandGroup(
                //new SetElevatorLedCommand(SuperState.eLow),
                new LowScoreSequence(mArmSubsystem, mElevatorSubsystem, mIntakeSubsystem)
            )
        );

        mButtonBox.humanPlayerPostion().onTrue(
            CommandFactory.HandleSuperStructureSequence(
                SuperState.ePlayerStation, mElevatorSubsystem, mArmSubsystem, mIntakeSubsystem
            )
        );

        mButtonBox.stowPostion().onTrue(
            CommandFactory
                .HandleSuperStructureSequence(SuperState.eStow, mElevatorSubsystem, mArmSubsystem, mIntakeSubsystem)
        );

        mButtonBox.cancel().onTrue(new InstantCommand(() -> {
            CommandScheduler.getInstance().cancelAll();
            ButtonBoxPublisher.enableLed(LedPosition.eCancel);
        }));

        // mButtonBox.cone1().onTrue(
        // new SetWaypointCommand(kPlacingPoses[0].robotPlacementPose, mDriveSubsystem,
        // LedPosition.eDriveTo1)
        // );

        // mButtonBox.cone1().onTrue(new HoldPositionCommand(mDriveSubsystem));

        mButtonBox.cube2().onTrue(
            new SetWaypointCommand(kPlacingPoses[1].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo2)
        );
        mButtonBox.cone3().onTrue(
            new SetWaypointCommand(kPlacingPoses[2].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo3)
        );

        mButtonBox.cone4().onTrue(
            new SetWaypointCommand(kPlacingPoses[3].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo4)
        );
        mButtonBox.cube5().onTrue(
            new SetWaypointCommand(kPlacingPoses[4].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo5)
        );
        mButtonBox.cone6().onTrue(
            new SetWaypointCommand(kPlacingPoses[5].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo6)
        );

        mButtonBox.cone7().onTrue(
            new SetWaypointCommand(kPlacingPoses[6].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo7)
        );
        mButtonBox.cube8().onTrue(
            new SetWaypointCommand(kPlacingPoses[7].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo8)
        );
        mButtonBox.cone9().onTrue(
            new SetWaypointCommand(kPlacingPoses[8].robotPlacementPose, mDriveSubsystem, LedPosition.eDriveTo9)
        );

        // TODO: Figure out stuff w human player coord left/right position
        // mButtonBox.leftHumanStation().onTrue(
        //     new SetWaypointCommand(LoadingZone.kDoubleSubstationPose, mDriveSubsystem, LedPosition.eHumanPlayerLeft)
        // );

        mButtonBox.leftHumanStation().onTrue(
            new FloorRetractSequence(mArmSubsystem, mElevatorSubsystem, mIntakeSubsystem)
        );

        mButtonBox.rightHumanStation().onTrue( new SequentialCommandGroup(
            new FloorPickupSequence(mArmSubsystem, mElevatorSubsystem, mIntakeSubsystem),
            new PrintToConsoleCommand("FLoor retract sequence initiated")
        ));
        // Function<Boolean, InstantCommand> jogElevatorCommand = (isUp) -> new
        // InstantCommand(() -> {
        // if (mButtonBox.isJoystickEnabled()) mElevatorSubsystem.jogElevator(isUp);
        // });

        Function<Boolean, InstantCommand> jogArmCommand = (isOut) -> new InstantCommand(() -> {
            mArmSubsystem.jogArm(isOut);
        });

        // mButtonBox.upJoystickButton().onTrue(jogElevatorCommand.apply(true)).onFalse(new
        // InstantCommand(() -> mElevatorSubsystem.stop()));
        // mButtonBox.downJoystickButton().onTrue(jogElevatorCommand.apply(false)).onFalse(new
        // InstantCommand(() -> mElevatorSubsystem.stop()));
        // mButtonBox.rightJoystickButton().onTrue(jogArmCommand.apply(true)).onFalse(new
        // InstantCommand(() -> mArmSubsystem.stop()));
        // mButtonBox.leftJoystickButton().onTrue(jogArmCommand.apply(false)).onFalse(new
        // InstantCommand(() -> mArmSubsystem.stop()));

        // mButtonBox.povUp().whileTrue(new JogElevatorCommand(mElevatorSubsystem,
        // 0.5));

        // mButtonBox.povDown().onTrue(new JogElevatorCommand(mElevatorSubsystem,
        // -0.5));

        // mButtonBox.povUp().onTrue(jogElevatorCommand.apply(true))
        // .onFalse(new InstantCommand(() -> mElevatorSubsystem.stop()));

        mButtonBox.povLeft().onTrue(jogArmCommand.apply(false)).onFalse(new InstantCommand(() -> mArmSubsystem.stop()));

        mButtonBox.povRight().onTrue(jogArmCommand.apply(true)).onFalse(new InstantCommand(() -> mArmSubsystem.stop()));

        // mButtonBox.povDown().onTrue(jogElevatorCommand.apply(false))
        // .onFalse(new InstantCommand(() -> mElevatorSubsystem.stop()));

        // mButtonBox.povUp().onTrue(jogElevatorCommand.apply(true))
        // .onFalse(new InstantCommand(() -> mElevatorSubsystem.stop()));

        // mButtonBox.povLeft().onTrue(jogArmCommand.apply(false)).onFalse(new
        // InstantCommand(() -> mArmSubsystem.stop()));

        // mButtonBox.povRight().onTrue(jogArmCommand.apply(true)).onFalse(new
        // InstantCommand(() -> mArmSubsystem.stop()));

        // mButtonBox.povDownLeft().onTrue(new InstantCommand(() -> {
        // if (mButtonBox.isJoystickEnabled()) {
        // mElevatorSubsystem.jogElevator(false);
        // mArmSubsystem.jogArm(false);
        // }
        // })).onFalse(new InstantCommand(() -> {
        // mElevatorSubsystem.stop();
        // mArmSubsystem.stop();
        // }));

        // mButtonBox.povDownRight().onTrue(new InstantCommand(() -> {
        // if (mButtonBox.isJoystickEnabled()) {
        // mElevatorSubsystem.jogElevator(false);
        // mArmSubsystem.jogArm(true);
        // }
        // })).onFalse(new InstantCommand(() -> {
        // mElevatorSubsystem.stop();
        // mArmSubsystem.stop();
        // }));

        // mButtonBox.povUpLeft().onTrue(new InstantCommand(() -> {
        // if (mButtonBox.isJoystickEnabled()) {
        // mElevatorSubsystem.jogElevator(true);
        // mArmSubsystem.jogArm(false);
        // }
        // })).onFalse(new InstantCommand(() -> {
        // mElevatorSubsystem.stop();
        // mArmSubsystem.stop();
        // }));

        // mButtonBox.povUpRight().onTrue(new InstantCommand(() -> {
        // if (mButtonBox.isJoystickEnabled()) {
        // mElevatorSubsystem.jogElevator(true);
        // mArmSubsystem.jogArm(true);
        // }
        // })).onFalse(new InstantCommand(() -> {
        // mElevatorSubsystem.stop();
        // mArmSubsystem.stop();
        // }));

        // mButtonBox.enableJoystick().onTrue(new InstantCommand(() ->
        // mButtonBox.setJoystickEnabled()))
        // .onFalse(new InstantCommand(() -> mButtonBox.setJoystickDisabled()));
        // mButtonBox.hook().onTrue(new InstantCommand(() ->
        // mArmSubsystem.setHookSpeed(0.2)))
        // .onFalse(new InstantCommand(() -> mArmSubsystem.stopHook()));

       // mButtonBox.hook().onTrue(new AttachHookCommand(mArmSubsystem)).onFalse(new DetachHookCommand(mArmSubsystem));

       mButtonBox.hook().onTrue(new InstantCommand(() -> mArmSubsystem.attachHook())).onFalse(new InstantCommand(() -> mArmSubsystem.detathHook()));

    }

    public static void setWasInFloor() {
        mWasInFloorPosition = true;
    }

    public static void setNotInFloorAnymore() {
        mWasInFloorPosition = false;
    }

    public static boolean getWasInFloor() {
        return mWasInFloorPosition;
    }

    public static void setIsCone() {
        mIsCone = true;
        ButtonBoxPublisher.enableLed(LedPosition.eConeActive);
    }

    public static void setIsCube() {
        mIsCone = false;
        ButtonBoxPublisher.enableLed(LedPosition.eCubeActive);
    }

    public static void toggleIsCone() {
        mIsCone = !mIsCone;

        LedPosition position;
        if (mIsCone) {
            position = LedPosition.eConeActive;
        }
        else {
            position = LedPosition.eCubeActive;
        }

        ButtonBoxPublisher.enableLed(position);
    }

    public static boolean isCone() {
        return mIsCone;
    }

    public static void setBackUpControllState() {
        mControllState = ControllState.eBackUpController;

    }

    public static void setAutomaticControllState() {
        mControllState = ControllState.eAutomaticControl;
    }

    public static void setManualControllState() {
        mControllState = ControllState.eManualControl;
    }

    public static ControllState getControllState() {
        return mControllState;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
     
        //return new LinearDriveCommand(mDriveSubsystem, 0.5, CardinalDirection.eY);
        //return new CubeHighStartingRight(mDriveSubsystem, mElevatorSubsystem, mArmSubsystem, mIntakeSubsystem);
        //return new CubeLowThenChargeStartingMiddle(mDriveSubsystem, mElevatorSubsystem, mArmSubsystem, mIntakeSubsystem);
        //return new BalanceRobotCommand(mDriveSubsystem);
        //return new BalanceRobotTimeIntervalCommand(mDriveSubsystem);
        //return new BalanceRobotForceDampCommand(mDriveSubsystem);

        //USe this for QUAL 61
        return new SequentialCommandGroup(
            new CubeLowOnly(mDriveSubsystem, mElevatorSubsystem, mArmSubsystem, mIntakeSubsystem),
            //new CubeHighOnly(mDriveSubsystem, mElevatorSubsystem, mArmSubsystem, mIntakeSubsystem),
            //new BalanceRobotTimeIntervalCommand(mDriveSubsystem)
            new TaxiRight(mDriveSubsystem, mElevatorSubsystem, mArmSubsystem, mIntakeSubsystem).withTimeout(2.25)
        );
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

    public void syncLeds() {
        int[] pressedButtons = mButtonBox.getPressedButtons();
        ButtonBoxPublisher.disableAllLeds(); // TODO if leds don't work, try commenting this out
        for (int button : pressedButtons) {
            ButtonPosition position = ButtonPosition.getButtonPosition(button);
            LedPosition led = kButtonBoxButtonMap.get(position);
            ButtonBoxPublisher.enableLed(led);

            if (ButtonPosition.getButtonPosition(button) == ButtonPosition.eConeActive) {
                RobotContainer.setIsCone();
            }
            else if (ButtonPosition.getButtonPosition(button) == ButtonPosition.eCubeActive) {
                RobotContainer.setIsCube();
            }
        }
    }

    public SubsystemManager getSubsystemManager() {
        return mSubsystemManager;
    }

    public static RobotContainer getInstance() {
        if (mInstance == null) mInstance = new RobotContainer();
        return mInstance;
    }
}
