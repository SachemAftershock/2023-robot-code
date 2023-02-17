package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.enums.ArmState;

public class ArmSubsystem extends TrapezoidProfileSubsystem{

    public ArmSubsystem() {
        super(new TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocityRadPerSecond, ArmConstants.kMaxAccelerationRadPerSecSquared),
            ArmConstants.kArmOffsetRads);
    }

    
    @Override
    public void useState(TrapezoidProfile.State setpoint) {

    }

    public ArmState getState() {
        return ArmState.eStow;
    }
    
}



