// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.SubsystemManager;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.*;
import frc.robot.commands.intake.IngestConeCommand;
import frc.robot.commands.intake.IngestCubeCommand;
import frc.robot.commands.intake.OutputConeCommand;
import frc.robot.commands.intake.OutputCubeCommand;
import frc.robot.commands.intake.StopIntakeCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static boolean mIsCone = true;
  
  private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();

  private final SubsystemManager mSubsystemManager = new SubsystemManager(mElevatorSubsystem, mIntakeSubsystem);

  private final Joystick mPrimaryThrottleController = new Joystick(ControllerConstants.kPrimaryThrottleControllerPort);
  private final Joystick mPrimaryTwistController = new Joystick(ControllerConstants.kPrimaryTwistControllerPort);
  private final ButtonBox mButtonBox = new ButtonBox(ControllerConstants.kButtonBoxPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();
  }

  public void initializeSubsystems() {
    mSubsystemManager.initialize();
  }

  private void configureBindings() {
    mButtonBox.cubeToggle().onTrue(
      new InstantCommand(() -> RobotContainer.toggleIsCone())
    );

    mButtonBox.ingestIntake().onTrue(
      RobotContainer.isCone() ? 
            new IngestConeCommand(mIntakeSubsystem) :
            new IngestCubeCommand(mIntakeSubsystem)
      ).onFalse(new StopIntakeCommand(mIntakeSubsystem));

    mButtonBox.ejectIntake().onTrue(
          RobotContainer.isCone() ? 
            new OutputConeCommand(mIntakeSubsystem) :
            new OutputCubeCommand(mIntakeSubsystem)
      ).onFalse(new StopIntakeCommand(mIntakeSubsystem));

    // mButtonBox.ingestIntake().onTrue(new
    // IngestCubeCommand(mIntakeSubsystem)).onFalse(new
    // StopIntakeCommand(mIntakeSubsystem));
    // mButtonBox.ejectIntake().onTrue(new
    // OutputCubeCommand(mIntakeSubsystem)).onFalse(new
    // StopIntakeCommand(mIntakeSubsystem));

    // mButtonBox.highPosition().onTrue(new ElevatorCommand(mElevatorSubsystem,
    // ElevatorPosition.HIGH)).onFalse(new StopIntakeCommand(mIntakeSubsystem));
    /**
     * mButtonBox.mediumPosition().onTrue(new ElevatorCommand(mElevatorSubsystem,
     * ElevatorPosition.MID )).onFalse(new StopIntakeCommand(mIntakeSubsystem));
     * mButtonBox.floorPosition().onTrue(new ElevatorCommand(mElevatorSubsystem,
     * ElevatorPosition.FLOOR )).onFalse(new StopIntakeCommand(mIntakeSubsystem));
     * mButtonBox.humanPlayerPostion().onTrue(new
     * ElevatorCommand(mElevatorSubsystem, ElevatorPosition.HUMANPlay)).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem));
     * mButtonBox.stowPostion().onTrue(new ElevatorCommand(mElevatorSubsystem,
     * ElevatorPosition.STOW )).onFalse(new StopIntakeCommand(mIntakeSubsystem)); //
     * SmartDashboard.putNumber("cone intake",
     * XboxController.Button.kRightBumper.value);
     */

    /**
     * mButtonBox.cone1().onTrue(new
     * ElevatorCommand(mElevatorSubsystem,mCurrentState )).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem)); // SmartDashboard.putNumber("cone
     * intake", XboxController.Button.kRightBumper.value);
     * mButtonBox.cube2().onTrue(new
     * ElevatorCommand(mElevatorSubsystem,mCurrentState )).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem)); // SmartDashboard.putNumber("cone
     * intake", XboxController.Button.kRightBumper.value);
     * mButtonBox.cone3().onTrue(new
     * ElevatorCommand(mElevatorSubsystem,mCurrentState )).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem)); // SmartDashboard.putNumber("cone
     * intake", XboxController.Button.kRightBumper.value);
     * 
     * mButtonBox.cone4().onTrue(new
     * ElevatorCommand(mElevatorSubsystem,mCurrentState )).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem)); // SmartDashboard.putNumber("cone
     * intake", XboxController.Button.kRightBumper.value);
     * mButtonBox.cube5().onTrue(new
     * ElevatorCommand(mElevatorSubsystem,mCurrentState )).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem)); // SmartDashboard.putNumber("cone
     * intake", XboxController.Button.kRightBumper.value);
     * mButtonBox.cone6().onTrue(new
     * ElevatorCommand(mElevatorSubsystem,mCurrentState )).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem)); // SmartDashboard.putNumber("cone
     * intake", XboxController.Button.kRightBumper.value);
     * 
     * mButtonBox.cone7().onTrue(new
     * ElevatorCommand(mElevatorSubsystem,mCurrentState )).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem)); // SmartDashboard.putNumber("cone
     * intake", XboxController.Button.kRightBumper.value);
     * mButtonBox.cube8().onTrue(new
     * ElevatorCommand(mElevatorSubsystem,mCurrentState )).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem)); // SmartDashboard.putNumber("cone
     * intake", XboxController.Button.kRightBumper.value);
     * mButtonBox.cone9().onTrue(new
     * ElevatorCommand(mElevatorSubsystem,mCurrentState )).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem));
     */

    /**
     * mButtonBox.leftHumanStation().onTrue(new ElevatorCommand(mElevatorSubsystem,
     * ElevatorPosition.HUMANStation )).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem));
     * mButtonBox.rightHumanStation().onTrue(new ElevatorCommand(mElevatorSubsystem,
     * ElevatorPosition.HUMANStation)).onFalse(new
     * StopIntakeCommand(mIntakeSubsystem));
     * 
     * mButtonBox.cancel().onTrue(new ElevatorCommand(mElevatorSubsystem,
     * ElevatorPosition.CANCEL )).onFalse(new StopIntakeCommand(mIntakeSubsystem));
     * // SmartDashboard.putNumber("cone intake",
     * XboxController.Button.kRightBumper.value);
     */
  }
  
  public static void toggleIsCone() {
    mIsCone = !mIsCone;
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
    return null;
  }
}