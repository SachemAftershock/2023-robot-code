package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.Constants.ArmConstants;

public class ArmSubsystem extends TrapezoidProfileSubsystem{

    public ArmSubsystem() {
        super(new TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared),
            ArmConstants.kArmOffsetRads);
    }

    
    @Override
    public void useState(TrapezoidProfile.State setpoint) {

    }
    
}



