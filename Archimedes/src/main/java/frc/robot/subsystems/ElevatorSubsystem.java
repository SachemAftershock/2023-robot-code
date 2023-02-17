package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;
import frc.lib.PID;
import frc.robot.RobotContainer;
import frc.robot.enums.ElevatorState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Ports.ElevatorPorts.*;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends AftershockSubsystem {

    private Lidar mLidar;
    private PID mPID;
    private CANSparkMax mMotor;

    private ElevatorState mCurrentState;
    private ElevatorState mDesiredState;

    public ElevatorSubsystem() {
        mLidar = new Lidar(kElevatorLidarId);
        mPID = null;
        mMotor = new CANSparkMax(kElevatorMotorId, MotorType.kBrushless);

        mCurrentState = ElevatorState.eStow;
        mDesiredState = ElevatorState.eStow;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {
        if (mCurrentState == mDesiredState)
            return;

        if (mPID == null) {
            mPID = new PID();
            mPID.start(kPidGains);
        }

        double setpoint = RobotContainer.isCone() ? mDesiredState.getConeHeight() : mDesiredState.getCubeHeight();
        double current = mLidar.getDistanceIn();

        if (Math.abs(current - setpoint) > kEpsilon) {
            stop();
            mCurrentState = mDesiredState;
            mPID = null;
            return;
        }

        double speed = mPID.update(current, setpoint);
        setSpeed(speed);
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
}
