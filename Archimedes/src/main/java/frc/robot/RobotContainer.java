// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.AftershockXboxController;
import frc.lib.Lidar;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.OperatorConstants;
// moved import frc.robot.Constants.ControllerConstants; to intakeSubsystem
//import frc.robot.commands.Autos;
//import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;

import frc.robot.subsystems.IntakeSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  //put in robot periodic

  // The robot's subsystems and commands are defined here...
  

  final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.kPrimaryControllerPort);

  /**private JoystickButton bInputCube = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
  private JoystickButton bOutputCube = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);

  private JoystickButton bInputCone = new JoystickButton(m_driverController, XboxController.Axis.kLeftTrigger.value);
  private JoystickButton bOutputCone = new JoystickButton(m_driverController, XboxController.Axis.kRightTrigger.value);**/

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /**new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));**/

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
       
    //m_driverController.rightBumper().onTrue(new IngestCubeCommand(mIntakeSubsystem)).onFalse(new StopIntakeCommand(mIntakeSubsystem));
   // m_driverController.leftBumper().onTrue(new OutputCubeCommand(mIntakeSubsystem)).onFalse(new StopIntakeCommand(mIntakeSubsystem));
    m_driverController.rightTrigger().onTrue(new IngestConeCommand(mIntakeSubsystem)).onFalse(new StopIntakeCommand(mIntakeSubsystem));
   // m_driverController.leftTrigger().onTrue(new OutputConeCommand(mIntakeSubsystem)).onFalse(new StopIntakeCommand(mIntakeSubsystem));

    //m_driverController.button(1).onTrue(new ElevatorCommand()).onFalse(new ElevatorCommand(mElevatorSubsystem));
   // SmartDashboard.putNumber("cone intake", XboxController.Button.kRightBumper.value);    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /* public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
*/
}