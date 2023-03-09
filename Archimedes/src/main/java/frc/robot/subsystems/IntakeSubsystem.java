package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Ports.IntakePorts.*;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports.IntakePorts;

public class IntakeSubsystem extends AftershockSubsystem {
    private static IntakeSubsystem mInstance;
    private final Lidar mLidar;
    

    private CANSparkMax mIntakeMotor = new CANSparkMax(kIntakeMotorId, MotorType.kBrushless);

    private IntakeSubsystem() {
        super();
        mLidar = new Lidar(new DigitalInput(IntakePorts.kIntakeLidarId));
    }

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        double lidarDistance = mLidar.getDistanceIn();
        if(lidarDistance < IntakeConstants.kIntakeWidth) {
            stop();
        }
    }

    @Override
    public void outputTelemetry() {

        SmartDashboard.putNumber("Intake Lidar Distance", mLidar.getDistanceIn());
        SmartDashboard.putNumber("Intake Motor Velocity", mIntakeMotor.getEncoder().getVelocity());
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
        System.out.println("Intake motor being called");
        mIntakeMotor.set(speed);
    }

    @Override
    public boolean checkSystem() {

        boolean isFunctional = false;
        double lidarDistance = mLidar.getDistanceIn();

        if(lidarDistance > 12 || lidarDistance == 0 || lidarDistance >= 999.0) {
            isFunctional = true;
        } else {
            System.out.println("ERROR : Intake Lidar not functional or misaligned. Lidar distance = " + lidarDistance);
        }
        return isFunctional;
    }

    public synchronized static IntakeSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSubsystem();
        }

        return mInstance;
    }
}
