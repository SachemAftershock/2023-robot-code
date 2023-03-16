package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
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

        mConstraints = new TrapezoidProfile.Constraints(kMaxVelocityMeterPerSecond, kMaxAccelerationMetersPerSecondSquared);
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

        ControllState controlState = RobotContainer.getControllState();
        double current = getBarDistance();

        if (mSetpoint != current) {
            mArmMode = ArmMode.eIdle;
        }

        if (mDesiredState != mCurrentState) {
            mArmMode = ArmMode.ePIDControl;
        } else {
            mArmMode = ArmMode.eIdle;
        }

        if (controlState == ControllState.eBackUpController || controlState == ControllState.eManualControl) {
            mArmMode = ArmMode.eManualControl;
        }

        if (mSetpoint < kMinArmBarDistance || mSetpoint > kMaxArmBarDistance) {
            System.out.println("Arm SETPOINT out of bounds: " + mSetpoint);
            stop();
            return;
        }

        //System.out.println("Desired state --> " + mDesiredState.toString() + " Current state --> " + mCurrentState.toString());

        switch (mArmMode) {
            case eStowedEmpty:
                //For having the initialize call bring the arm and elevator back in if they are out
                break;
            case eIdle: 

                if(mPID.isPaused()) mPID.resumePID();
                double idleOutput = mPID.update(current, mSetpoint);
                setSpeed(idleOutput);
                break;

            case ePIDControl:

                if(mPID.isPaused()) mPID.resumePID();

                if (mSetpoint == -1) {
                    System.out.println("ERROR : Arm setpoint invalid");
                    return;
                }

                current = getBarDistance();
                mSetpoint = mDesiredState.getLength();

                double output = mPID.update(current, mSetpoint);
                output = output * kArmSpeedScalingFactor;

                if (Math.abs(mPID.getError()) < kEpsilon) {
                    mCurrentState = mDesiredState;
                    mArmMode = ArmMode.eIdle;
                    return;
                }

                setSpeed(output);

                break;
            case eManualControl: 

                if(!(mPID.isPaused())) mPID.pausePID();
                setSetpoint(getBarDistance());
                
                break;
            default: 
                System.out.println("Arm in erraneous state");
                break;

        }
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
        return mFilter.calculate(mLidar.getDistanceIn() + kArmLidarOffset);
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
