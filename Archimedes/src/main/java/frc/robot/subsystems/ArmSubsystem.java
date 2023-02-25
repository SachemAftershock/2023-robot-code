package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.enums.ArmState;
import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Ports.ArmPorts.*;

public class ArmSubsystem extends AftershockSubsystem {

    private static ArmSubsystem mInstance;
    private CANSparkMax mArmMotor;

    private final Lidar mLidar;
    private final ProfiledPIDController mProfileController;
    private final TrapezoidProfile.Constraints mConstraints;

    private ArmState mCurrentState;
    private ArmState mDesiredState;

    private ArmSubsystem() {
        super();

        mArmMotor = new CANSparkMax(kArmMotorId, MotorType.kBrushless);
        mLidar = new Lidar(new DigitalInput(kArmLidarId));

        mConstraints = new TrapezoidProfile.Constraints(kMaxVelocityMeterPerSecond, kMaxAccelerationMetersPerSecondSquared);
        mProfileController = new ProfiledPIDController(kGains[0], kGains[1], kGains[2], mConstraints);

        mCurrentState = ArmState.eStow;
        mDesiredState = ArmState.eStow;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        if (mCurrentState == mDesiredState) return;

        double current = mLidar.getDistanceCm() / 100;
        double setpoint = mDesiredState.getLength();

        setSpeed(mProfileController.calculate(current, setpoint));

        if (Math.abs(current - setpoint) < kEpsilon) {
            stop();
            mCurrentState = mDesiredState;
        }
    }

    public void stop() {
        setSpeed(0);
    }

    public ArmState getState() {
        return mCurrentState;
    }

    public void setDesiredState(ArmState desiredState) {
        mDesiredState = desiredState;
    }

    @Override
    public void outputTelemetry() {

    }

    private void setSpeed(double speed) {
        mArmMotor.set(speed);
    }

    public static synchronized ArmSubsystem getInstance() {
        if (mInstance != null) mInstance = new ArmSubsystem();
        return mInstance;
    }
}
