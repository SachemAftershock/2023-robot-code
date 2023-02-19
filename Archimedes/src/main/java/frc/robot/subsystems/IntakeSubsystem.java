package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Ports.IntakePorts.*;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends AftershockSubsystem {
    private CANSparkMax mIntakeMotor = new CANSparkMax(kIntakeMotorId, MotorType.kBrushless);

    public IntakeSubsystem() {
        super();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void outputTelemetry() {
    }

    /**
     * Turn on the motor counter clockwise to intake the cone
     */
    public void ingestCone() {
        setSpeed(-(IntakeConstants.kIngestConeSpeed));
    }

    /**
     * Turn on the motor clockwise to output the cone
     */
    public void outputCone() {
        setSpeed(IntakeConstants.kIngestConeSpeed);
    }

    /**
     * Turn on the motor clockwise to intacke the cube
     */
    public void ingestCube() {
        setSpeed(IntakeConstants.kIngestCubeSpeed);
    }

    /**
     * Turn on the motor counter clockwise to output the cube
     */
    public void outputCube() {
        setSpeed(-(IntakeConstants.kIngestCubeSpeed));
    }

    public void stop() {
        setSpeed(0);
    }
    
    private void setSpeed(double speed) {
        mIntakeMotor.set(speed);
    }
}
