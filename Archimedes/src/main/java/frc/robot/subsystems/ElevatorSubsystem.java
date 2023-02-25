package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;
import frc.robot.enums.ElevatorState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.robot.Ports.ElevatorPorts.*;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends AftershockSubsystem {
    private static ElevatorSubsystem mInstance;

    private Lidar mLidar;
    private final ProfiledPIDController mProfileController;
    private final TrapezoidProfile.Constraints mConstraints;

    private CANSparkMax mMotor;

    private ElevatorState mCurrentState;
    private ElevatorState mDesiredState;

    private ElevatorSubsystem() {
        mLidar = new Lidar(new DigitalInput(kElevatorLidarId));
        mConstraints = new TrapezoidProfile.Constraints(kMaxVelocityMeterPerSecond, kMaxAccelerationMetersPerSecondSquared);
        mProfileController = new ProfiledPIDController(kPidGains[0], kPidGains[1], kPidGains[2], mConstraints);
        mMotor = new CANSparkMax(kElevatorMotorId, MotorType.kBrushless);

        mCurrentState = ElevatorState.eStow;
        mDesiredState = ElevatorState.eStow;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        if (mCurrentState == mDesiredState) return;

        double setpoint = mDesiredState.getHeight();
        double current = mLidar.getDistanceIn();

        setSpeed(mProfileController.calculate(current, setpoint));

        if (Math.abs(current - setpoint) < kEpsilon) {
            stop();
            mCurrentState = mDesiredState;
        }
    }

    public void setDesiredState(ElevatorState desiredState) {
        mDesiredState = desiredState;
    }

    public ElevatorState getState() {
        return mCurrentState;
    }

    public void stop() {
        setSpeed(0);
    }

    private void setSpeed(double speed) {
        mMotor.set(speed);
    }

    @Override
    public void outputTelemetry() {
    }

    public synchronized static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }

        return mInstance;
    }
}
