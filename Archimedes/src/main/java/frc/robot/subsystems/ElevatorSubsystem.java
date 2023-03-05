package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;
import frc.lib.PID;
import frc.robot.enums.ElevatorState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Ports.ElevatorPorts.*;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends AftershockSubsystem {
    private static ElevatorSubsystem mInstance;

    private Lidar mLidar;
    private ProfiledPIDController mProfileController;
    private final TrapezoidProfile.Constraints mConstraints;

    private CANSparkMax mMotor;

    private ElevatorState mCurrentState;
    private ElevatorState mDesiredState;

    ShuffleboardTab ElevatorSubsystemTab = Shuffleboard.getTab("Elevator Subsystem");
    GenericEntry P = ElevatorSubsystemTab.add("Elevator P", 0).getEntry();
    GenericEntry I = ElevatorSubsystemTab.add("Elevator I", 0).getEntry();
    GenericEntry D = ElevatorSubsystemTab.add("Elevator D", 0).getEntry();
    PID mPid = new PID();

    public boolean breakThis = false;

    private ElevatorSubsystem() {
        mLidar = new Lidar(new DigitalInput(kElevatorLidarId));
        mConstraints = new TrapezoidProfile.Constraints(kMaxVelocityMeterPerSecond, kMaxAccelerationMetersPerSecondSquared);
        mProfileController = new ProfiledPIDController(kPidGains[0], kPidGains[1], kPidGains[2], mConstraints);
        mMotor = new CANSparkMax(kElevatorMotorId, MotorType.kBrushless);
        mMotor.setIdleMode(IdleMode.kBrake);

        mCurrentState = ElevatorState.eStow;
        mDesiredState = ElevatorState.eStow;

        mPid.start(kPidGains);
    }

    @Override
    public void initialize() {
        mProfileController = new ProfiledPIDController(P.getDouble(0), I.getDouble(0), D.getDouble(0), mConstraints);
    }

    @Override
    public void periodic() {
        if (mCurrentState == mDesiredState) return;
        //if (breakThis) return;

        double setpoint = mDesiredState.getHeight();
        double current = mLidar.getDistanceIn();
        double output = mPid.update(current, setpoint);//mProfileController.calculate(current, setpoint);

        System.out.println("Current " + current + " SetPoint " + setpoint + " Output " + output);
        setSpeed(-output);

        if (Math.abs(current - setpoint) < kEpsilon) {
            stop();
            mCurrentState = mDesiredState;
        }
    }

    public void setBreakTrue() {
        breakThis = true;
    }

    public void setBreakFalse() {
        breakThis = false;
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

    public void jogElevator(boolean moveUp) {
        setSpeed(moveUp ? kJogSpeed : -kJogSpeed);
    }

    private void setSpeed(double speed) {
        mMotor.set(speed);
    }

    public double getElevatorDistance() {
        return mLidar.getDistanceIn() + kElevatorLidarOffset;
    }

    public void setTestSpeed(double speed) {
        mMotor.set(speed);
    }

    @Override
    public void outputTelemetry() {
        // ElevatorSubsystemTab.add("E Lidar Distance Inches", mLidar.getDistanceCm());
        // ElevatorSubsystemTab.add("E Motor Velocity", mMotor.getEncoder().getVelocity());

        //SmartDashboard.putNumber("Raw Distance", mLidar.getDistanceIn());
        SmartDashboard.putNumber("Elevator Distance", getElevatorDistance());
        SmartDashboard.putNumber("Elevator Motor Velocity", mMotor.getEncoder().getVelocity());
    }

    public synchronized static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }

        return mInstance;
    }
}
