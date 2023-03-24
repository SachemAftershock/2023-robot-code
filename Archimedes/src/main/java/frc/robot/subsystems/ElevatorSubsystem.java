package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;
import frc.lib.PID;
import frc.robot.RobotContainer;
import frc.robot.enums.ControllState;
import frc.robot.ErrorTracker;
import frc.robot.ErrorTracker.ErrorType;
import frc.robot.enums.ElevatorState;

import com.ctre.phoenixpro.Timestamp;
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

import javax.swing.border.EmptyBorder;

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
    private double prevTestDistance;
    private double mSystemTimer;

    private ElevatorMode mElevatorMode;

    public enum ElevatorMode {
        eStowedEmpty, eIdle, ePIDControl, eManualControl, eRewinding, eBadState
    }

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
        mElevatorMode = ElevatorMode.eStowedEmpty;

        mFilter = new MedianFilter(kElevatorMedianFilterSampleSize);
    }

    @Override
    public void initialize() {

        mCurrentState = ElevatorState.eStowEmpty;
        mDesiredState = ElevatorState.eStowEmpty;
        mSetpoint = getElevatorHeight(); // Temporary so Elevator doesnt move when enabled
        // mProfileController = new ProfiledPIDController(kPidGains[0], kPidGains[1],
        // kPidGains[2], mConstraints);
        setSpeed(0);
        counter = 0;
        prevDelta = Double.MAX_VALUE;
        prevTestDistance = getElevatorHeight();
    }

    @Override
    public void periodic() {
        //System.out.println(getElevatorHeight());

        ControllState controlState = RobotContainer.getControllState();
        double current = getElevatorHeight();
        double speed = mMotor.get();

        if(mSetpoint != current) {
            mElevatorMode = ElevatorMode.eIdle;
        }

        //System.out.println("Desired state --> " + mDesiredState.toString() + " Current state --> " + mCurrentState.toString());

        if(mDesiredState != mCurrentState) {
            mElevatorMode = ElevatorMode.ePIDControl;
        } else {
            mElevatorMode = ElevatorMode.eIdle;
        }

        if(controlState == ControllState.eBackUpController || controlState == ControllState.eManualControl) {
            mElevatorMode = ElevatorMode.eManualControl;
        }

        if (mSetpoint > kElevatorMaxHeight || mSetpoint < kElevatorMinHeight || Double.isNaN(mSetpoint)) {
            System.out.println("Elevator SETPOINT out of bounds: " + mSetpoint);
            stop();
            return;
        }

        //System.out.println("Current mode --> " + mElevatorMode);
        
        //Setpoint should get set by this state machine
        //eIdle and ePIDControl is seperate in case to lock in setpoint
        //Elevator state machine sets the setpoint and starts the PID

        switch (mElevatorMode) {
            case eStowedEmpty:
                break;

            case eIdle:
                //Runs the pid but only to hold it in place
                // if(mPid.isPaused()) mPid.resumePID();
                // checkBounds(current);
                // double idleOutput = mPid.update(current, mSetpoint);
                // setSpeed(idleOutput);

                mMotor.setVoltage(0.5); //was 0.5

                break;

            case ePIDControl:
                //Only be in this state if the elevator is moving
                if(mPid.isPaused()) mPid.resumePID();
                checkBounds(current);
                mSystemTimer = System.currentTimeMillis();
                //This time is in miliseconds
                //Was 1000 before
                // if(mSystemTimer > 100) {
                //     double currentTestDistance = getElevatorHeight();
                //     if(Math.abs(speed) > 0.0 && Math.abs(currentTestDistance - prevTestDistance) > kEpsilon) {
                //         System.out.println("ERROR : ---- Elevator Wound Backwards ----" + " speed (in RPM) --> " + 
                //         speed + " Distance delta --> " + Math.abs(currentTestDistance - prevTestDistance));
                //         stop();
                //         mElevatorMode = ElevatorMode.eRewinding;
                //     }
                // }

                double output = mPid.update(current, mSetpoint);

                if (Double.isNaN(output)) {
                    System.out.println("Output NaN");
                    return;
                }

                if (Math.abs(mPid.getError()) < kEpsilon) {
                    mCurrentState = mDesiredState;
                    mElevatorMode = ElevatorMode.eIdle;
                    return;
                }

                //System.out.println("Setpoint --> " + mSetpoint + " Current --> " + current + " speed --> " + output);
                output = output*0.9;
                setSpeed(output);

                break;

            case eManualControl:
                //Pauses the PID and re-engages it once manual control is released
                if(!(mPid.isPaused())) mPid.pausePID();
                // Use this check if driver is stupid and unwinds the rope
                // if(mSystemTimer > 200) {
                //     double currentTestDistance = getElevatorHeight();
                //     if(Math.abs(speed) > 0.0 &&  Math.abs(currentTestDistance - prevTestDistance) < kEpsilon) {
                //         System.out.println("ERROR : ---- Elevator Wound Backwards ----" + " speed --> " + 
                //         speed + " Distance delta --> " + Math.abs(currentTestDistance - prevTestDistance));
                //         stop();
                //         mElevatorMode = ElevatorMode.eRewinding;
                //     }
                //     prevTestDistance = currentTestDistance;
                // }

                //System.out.println("Manual getting called");
                
                setSetpoint(getElevatorHeight());

                break;

            case eRewinding:
                //If an unwind has been detected elevator will automatically go into this state 
                //and attempt to rewind the rope
                mPid.pausePID();
                // mSystemTimer = System.currentTimeMillis();
                // if(mSystemTimer > 100) {
                //     double currentTestDistance = getElevatorHeight();
                //     if(Math.abs(speed) > 0.0 &&  Math.abs(currentTestDistance - prevTestDistance) < kEpsilon) {
                //         setSpeed(-0.2);
                //     } else {
                //         stop();
                //     }
                //     prevTestDistance = currentTestDistance;
                // }
                // stop();
                // mElevatorMode = ElevatorMode.eRewinding;
            break;
                
            default:
                System.out.println("------ Elevator in erraneous state ------");
                break;
        }

    }

    public void checkBounds(double current) {
        if (current > kElevatorMaxHeight || current < kElevatorMinHeight) {
            System.out.println("ERROR ---------- ELEVATOR OUT OF BOUNDS  ----------");
            stop();
            return;
        }
    }

    public void setDesiredState(ElevatorState desiredState) {
        mDesiredState = desiredState;
        setSetpoint(mDesiredState.getHeight());
    }

    public void setSetpoint(double setpoint) {
        mSetpoint = setpoint;
    }

    public ElevatorState getState() {
        return mCurrentState;
    }

    public boolean isLowEnough() {
        return getElevatorHeight() <= 41;
    }

    public void jogSetpoint(double jogValue) {
        System.out.println("Jog value --> " + jogValue + "  Setpoint --> " + mSetpoint);
        mSetpoint += jogValue;
    }

    public void stop() {
        // System.out.println("Stopping elevator");
        setSpeed(0);
    }

    public void jogElevator(boolean moveUp) {
        setSpeed(moveUp ? kJogSpeed : -kJogSpeed);
    }

    public void jogElevatorUp() {
        setSpeed(0.6);
    }

    public void jogElevatorDown() {
        setSpeed(-0.6);

    }

    private void setSpeed(double speed) {
        if (getElevatorHeight() > kElevatorMaxHeight || getElevatorHeight() < kElevatorMinHeight) {
            DriverStation.reportError("ELEVATOR OUT OF BOUNDS (set speed call)", false);
            return;
        }
        else {
            //System.out.println("Speed --> " + speed);
            mMotor.set(speed);
        }
    }

    public void setManualSpeed(double speed) {
        setSpeed(speed);
    }

    public double getElevatorDistance() {
        double distance = mLidar.getDistanceIn() + kElevatorLidarOffset;

        ErrorTracker tracker = ErrorTracker.getInstance();
        if (Double.isNaN(distance)) tracker.enableError(ErrorType.eElevatorLidarInfinity);
        else if (tracker.isErrorEnabled(ErrorType.eElevatorLidarInfinity)) {
            tracker.disableError(ErrorType.eElevatorLidarInfinity);
        }

        return distance;
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
        SmartDashboard.putNumber("Elevator Distance", getElevatorHeight());
        SmartDashboard.putString("Elevator Mode ", mElevatorMode.toString());
    }

    public synchronized static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }

        return mInstance;
    }
}
