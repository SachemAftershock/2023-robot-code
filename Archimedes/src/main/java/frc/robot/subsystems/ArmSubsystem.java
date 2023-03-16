package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.ErrorTracker;
import frc.robot.ErrorTracker.ErrorType;
import frc.robot.enums.ArmState;
import frc.robot.enums.ControllState;
import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;
import frc.lib.PID;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Ports.ArmPorts.*;

public class ArmSubsystem extends AftershockSubsystem {

    private static ArmSubsystem mInstance;
    private CANSparkMax mArmMotor;
    private TalonSRX mHookMotor;

    private final Lidar mLidar;
    private ProfiledPIDController mProfileController; // Add final back
    private final TrapezoidProfile.Constraints mConstraints;
    private MedianFilter mFilter;
    private PID mPID;

    private ArmState mCurrentState;
    private ArmState mDesiredState;
    public boolean mBreak = false;

    private ArmMode mArmMode;
    private double mSetpoint;

    ShuffleboardTab ArmSubsystemTab = Shuffleboard.getTab("Arm Subsystem");
    GenericEntry P = ArmSubsystemTab.add("Arm P", 0).getEntry();
    GenericEntry I = ArmSubsystemTab.add("Arm I", 0).getEntry();
    GenericEntry D = ArmSubsystemTab.add("Arm D", 0).getEntry();

    public enum ArmMode {
        ePIDControl, eManualControl, eIdle, eStowedEmpty, eLocked;
    }

    private ArmSubsystem() {
        super();

        mArmMotor = new CANSparkMax(kArmMotorId, MotorType.kBrushless);
        mHookMotor = new TalonSRX(kHookMotorId);
        mHookMotor.setNeutralMode(NeutralMode.Brake);

        mLidar = new Lidar(new DigitalInput(kArmLidarId));

        mConstraints = new TrapezoidProfile.Constraints(
            kMaxVelocityMeterPerSecond, kMaxAccelerationMetersPerSecondSquared
        );
        mProfileController = new ProfiledPIDController(kGains[0], kGains[1], kGains[2], mConstraints);
        mFilter = new MedianFilter(10);
        mPID = new PID();

        mCurrentState = ArmState.eStowEmpty;
        mDesiredState = ArmState.eStowEmpty;

        mArmMode = ArmMode.eStowedEmpty;
    }

    @Override
    public void initialize() {
        // mProfileController = new ProfiledPIDController(P.getDouble(0),
        // I.getDouble(0), D.getDouble(0), mConstraints);
        setSpeed(0);
        mCurrentState = null;
        mDesiredState = null;
        mPID.start(kGains);
        mSetpoint = getBarDistance();

    }

    @Override
    public void periodic() {

        if (mCurrentState == mDesiredState) {
            // System.out.println("Desired state reached");
            // System.out.println("----------PID ERROR------------" + mPID.getError());
            return;
        }

        double current = mFilter.calculate(getBarDistance());
        double setpoint = mDesiredState.getLength();// ArmConstants.getBarDistance(mDesiredState.getLength());
        // System.out.println(setpoint + ", " + mDesiredState.getLength());
        if (setpoint == -1) {
            System.out.println("ERROR : Arm setpoint invalid");
            // DriverStation.reportError("[INTAKE]: SETPOINT IS INVALID", false);
            return;
        }

        double output = mPID.update(current, setpoint);
        // output = MathUtil.clamp(output, -0.5, 0.5);
        output = output * 0.4;

        // System.out.println("Current " + current + " SetPoint " + setpoint + " Output
        // " + output);
        if (Math.abs(mPID.getError()) < kEpsilon) {
            System.out.println("-----EXITING PID-----" + mPID.getError());
            mCurrentState = mDesiredState;
            stop();
            return;
        }

        setSpeed(output);

    }

    public void setSetpoint(double setpoint) {
        mSetpoint = setpoint;
    }

    public void setDesiredState(ArmState desiredState) {
        mDesiredState = desiredState;
        setSetpoint(mDesiredState.getLength());
    }

    public void stop() {
        setSpeed(0);
    }

    public void jogArm(boolean isOut) {
        setSpeed(isOut ? -kJogSpeed : kJogSpeed);
    }

    public void jogArmOut() {
        setSpeed(-0.2);
    }

    public void jogArmIn() {
        setSpeed(0.2);

    }

    public ArmState getState() {
        return mCurrentState;
    }

    public double getBarDistance() {
        double distance = mLidar.getDistanceIn() + kArmLidarOffset;

        ErrorTracker tracker = ErrorTracker.getInstance();
        if (Double.isNaN(distance)) tracker.enableError(ErrorType.eArmLidarInfinity);
        else if (tracker.isErrorEnabled(ErrorType.eArmLidarInfinity)) tracker.disableError(ErrorType.eArmLidarInfinity);

        return distance;
    }

    @Override
    public void outputTelemetry() {

        // ArmSubsystemTab.add("A Lidar Distance Inches", mLidar.getDistanceCm());
        // ArmSubsystemTab.add("A Real Bar Distance",
        // ArmConstants.getBarDistance(kArmLidarId));
        // ArmSubsystemTab.add("A Motor Velocity",
        // mArmMotor.getEncoder().getVelocity());

        // SmartDashboard.putNumber("Raw Bar Distance", mLidar.getDistanceIn());
        SmartDashboard.putNumber("Bar Distance", getBarDistance());
        SmartDashboard.putNumber("Arm Motor Velocity", mArmMotor.getEncoder().getVelocity());
    }

    @Override
    public boolean checkSystem() {

        boolean isFunctional = true;
        double lidarDistance = mLidar.getDistanceIn();

        // Value should be lidar distance when arm is fully retractred
        if (lidarDistance < 0) {
            System.out.println("ERROR : Arm Lidar not functional or misaligned. Lidar distance = " + lidarDistance);
            isFunctional = false;
        }

        return isFunctional;
    }

    public void overrideCurrentState() {
        mCurrentState = ArmState.eUnknown;
    }

    public void TESTSPEED() {
        setSpeed(-1.0);
    }

    private void setSpeed(double speed) {
        // System.out.println("Motor Controller speed" + speed);
        mArmMotor.set(speed);
    }

    public void setTestSpeed(double speed) {
        mArmMotor.set(speed);
    }

    public static synchronized ArmSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ArmSubsystem();
        }
        return mInstance;
    }
}
