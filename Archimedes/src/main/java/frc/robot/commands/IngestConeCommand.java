package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IngestConeCommand extends InstantCommand {
  private double previousEncoderPosition = 0;
  private IntakeSubsystem mIntakeSubsystem;
    
    
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IngestConeCommand(IntakeSubsystem intakeSubsystem) {
    mIntakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  //@Override
  public void execute() {
    double currentEncoderValue = mIntakeSubsystem.getEncoderValue();
    if(Math.abs(currentEncoderValue-previousEncoderPosition) > Constants.kNEO550CountsperREV){
      //change call to getVelocity in getPosition thinkAabout
       mIntakeSubsystem.ingestCone();
    }
    else
    {
      mIntakeSubsystem.stopIntakeMotor();
    }
    previousEncoderPosition = currentEncoderValue;
  }
}