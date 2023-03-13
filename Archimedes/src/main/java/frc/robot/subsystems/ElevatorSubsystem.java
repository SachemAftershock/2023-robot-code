package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;
import frc.lib.PID;
import frc.robot.enums.ElevatorState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
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

    private PID mPid;

    private CANSparkMax mMotor;

    private ElevatorState mCurrentState;
    private ElevatorState mDesiredState;
    private final MedianFilter mFilter;

    ShuffleboardTab ElevatorSubsystemTab = Shuffleboard.getTab("Elevator Subsystem");
    GenericEntry P = ElevatorSubsystemTab.add("Elevator P", 0).getEntry();
    GenericEntry I = ElevatorSubsystemTab.add("Elevator I", 0).getEntry();
    GenericEntry D = ElevatorSubsystemTab.add("Elevator D", 0).getEntry();

    private double mSetpoint;
    private int counter;
    private double prevDelta;

    private ElevatorSubsystem() {

        mLidar = new Lidar(new DigitalInput(kElevatorLidarId));
        mPid = new PID();
        mPid.start(kPidGains);
        mConstraints = new TrapezoidProfile.Constraints(
            kMaxVelocityMeterPerSecond, kMaxAccelerationMetersPerSecondSquared
        );
        mProfileController = new ProfiledPIDController(
            kTrapezoidalPidGains[0], kTrapezoidalPidGains[1], kTrapezoidalPidGains[2], mConstraints
        );
        mMotor = new CANSparkMax(kElevatorMotorId, MotorType.kBrushless);
        mMotor.setIdleMode(IdleMode.kBrake);
        // mMotor.setInverted(true);

        mCurrentState = ElevatorState.eStowEmpty;
        mDesiredState = ElevatorState.eStowEmpty;

        mFilter = new MedianFilter(kElevatorMedianFilterSampleSize);
    }

    @Override
    public void initialize() {

        // mCurrentState = ElevatorState.eStowEmpty;
        // mDesiredState = ElevatorState.eStowEmpty;
        mSetpoint = getElevatorHeight(); // Temporary so Elevator doesnt move when enabled
        // mProfileController = new ProfiledPIDController(kPidGains[0], kPidGains[1],
        // kPidGains[2], mConstraints);
        setSpeed(0);
        counter = 0;
        prevDelta = Double.MAX_VALUE;
    }

    @Override
    public void periodic() {

        if (DriverStation.isTest()) return;

        double current = getElevatorHeight();

        if (Math.abs(current - mSetpoint) < kEpsilon) {
            mCurrentState = mDesiredState;
            stop();
            return;
        }

        if (mSetpoint > kElevatorMaxHeight || mSetpoint < kElevatorMinHeight || Double.isNaN(mSetpoint)) {
            System.out.println("Elevator SETPOINT out of bounds: " + mSetpoint);
            stop();
            return;
        }

        if (current > kElevatorMaxHeight || current < kElevatorMinHeight) {
            System.out.println("ERROR ---------- ELEVATOR OUT OF BOUNDS  ----------");
            stop();
            DriverStation.reportError("ELEVATOR OUT OF BOUNDS", false);
            return;
        }

        double output = mPid.update(current, mSetpoint);

        // if (mCurrentState == mDesiredState && mCurrentState !=
        // ElevatorState.eStowEmpty) {
        // setVoltage(kCompensatingVoltage);
        // }

        // if (mCurrentState == mDesiredState) return;

        // double output = MathUtil.clamp(mProfileController.calculate(current,
        // setpoint), -1.0, 1.0);

        if (counter > 100) {
            double currentDelta = Math.abs(mPid.getError());
            if (currentDelta > prevDelta) {
                System.out.println(
                    "ERROR ---------- ELEVATOR ROPE WOUND BACKWARDS ----------" + mPid.getError() + "  " + prevDelta
                );
                // mCurrentState = null;
                // mDesiredState = null;
                // stop();
                // return;
            }
            prevDelta = currentDelta;
            counter = 0;
        }
        counter++;

        if (Double.isNaN(output)) {
            System.out.println("Output NaN");
            return;
        }

        setSpeed(output);
    }

    // public void setVoltage(double voltage) {
    // mMotor.setVoltage(voltage);
    // }

    public void setDesiredState(ElevatorState desiredState) {
        mSetpoint = desiredState.getHeight();
        mDesiredState = desiredState;
    }

    public ElevatorState getState() {
        return mCurrentState;
    }

    public void jogSetpoint(double jogValue) {
        mSetpoint += jogValue;
    }

    public void stop() {
        System.out.println("Stopping elevator");
        setSpeed(0);
    }

    public void jogElevator(boolean moveUp) {
        setSpeed(moveUp ? kJogSpeed : -kJogSpeed);
    }

    private void setSpeed(double speed) {
        if (getElevatorHeight() > kElevatorMaxHeight || getElevatorHeight() < kElevatorMinHeight) {
            DriverStation.reportError("ELEVATOR OUT OF BOUNDS", false);
            return;
        }
        else {
            mMotor.set(speed);
        }
    }

    public double getElevatorDistance() {
        return mLidar.getDistanceIn() + kElevatorLidarOffset;
    }

    public double getFilteredDistance() {
        return mFilter.calculate(mLidar.getDistanceIn()) + kElevatorLidarOffset;
    }

    public double getElevatorHeight() {
        return getFilteredDistance() + kElevatorLidarHeightFromGround;
    }

    public void setTestSpeed(double speed) {
        mMotor.set(speed);
    }

    @Override
    public void outputTelemetry() {
        // ElevatorSubsystemTab.add("E Lidar Distance Inches", mLidar.getDistanceCm());
        // ElevatorSubsystemTab.add("E Motor Velocity",
        // mMotor.getEncoder().getVelocity());

        // SmartDashboard.putNumber("Raw Distance", mLidar.getDistanceIn());
        SmartDashboard.putNumber("Elevator Distance", getElevatorDistance());
        SmartDashboard.putNumber("Filtered Elevator Distance", getFilteredDistance());
        SmartDashboard.putNumber("Elevator Motor Velocity", mMotor.getEncoder().getVelocity());
    }

    public synchronized static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }

        return mInstance;
    }
}
