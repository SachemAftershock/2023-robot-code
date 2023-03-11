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
import frc.robot.Constants.ArmConstants;
import frc.robot.enums.ArmState;
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
    private ProfiledPIDController mProfileController; //Add final back
    private final TrapezoidProfile.Constraints mConstraints;
    private MedianFilter mFilter;
    private PID mPID;

    private ArmState mCurrentState;
    private ArmState mDesiredState;
    public boolean mBreak = false;
    
    double i = 0;

    ShuffleboardTab ArmSubsystemTab = Shuffleboard.getTab("Arm Subsystem");
    GenericEntry P = ArmSubsystemTab.add("Arm P", 0).getEntry();
    GenericEntry I = ArmSubsystemTab.add("Arm I", 0).getEntry();
    GenericEntry D = ArmSubsystemTab.add("Arm D", 0).getEntry();

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
    }

    @Override
    public void initialize() {
        //mProfileController = new ProfiledPIDController(P.getDouble(0), I.getDouble(0), D.getDouble(0), mConstraints);
        setSpeed(0);
        mCurrentState = ArmState.eStowEmpty;
        mDesiredState = ArmState.eStowEmpty;
        mPID.start(kGains);

    }

    @Override
    public void periodic() {

        //System.out.println(mCurrentState + " " + mDesiredState);

        if (mCurrentState == mDesiredState) {
            //System.out.println("Desired state reached");
            //System.out.println("----------PID ERROR------------" + mPID.getError());
            return;
        }
        //if(mBreak || true) return;  

        double current = mFilter.calculate(getBarDistance());
        double setpoint = mDesiredState.getLength();//ArmConstants.getBarDistance(mDesiredState.getLength());
        //System.out.println(setpoint + ", " + mDesiredState.getLength());
        if (setpoint == -1) {
            System.out.println("ERROR : Arm setpoint invalid");
            //DriverStation.reportError("[INTAKE]: SETPOINT IS INVALID", false);
            return;
        }

        // if(current < ArmConstants.getBarDistance(66.0)) {
        //     stop();
        //     return;
        // }

        //double output = MathUtil.clamp(mProfileController.calculate(current, setpoint), -0.5, 0.5);
        double output = mPID.update(current, setpoint);
        //output = MathUtil.clamp(output, -0.5, 0.5);
        output = output*0.4;

        System.out.println("Current " + current + " SetPoint " + setpoint + " Output " + output);
        if (Math.abs(mPID.getError()) < kEpsilon) {
            System.out.println("-----EXITING PID-----" + mPID.getError());
            stop();
            mCurrentState = mDesiredState;
            return;
        }
        setSpeed(output);
        i++;
    }

    public void stop() {
        setSpeed(0);
    }

    public void jogArm(boolean isOut) {
        setSpeed(isOut ? kJogSpeed : -kJogSpeed);
    }

    public ArmState getState() {
        return mCurrentState;
    }

    public void setDesiredState(ArmState desiredState) {
        mDesiredState = desiredState;
    }

    public double getBarDistance() {
        return mLidar.getDistanceIn() + kArmLidarOffset;
    }

    @Override
    public void outputTelemetry() {

        // ArmSubsystemTab.add("A Lidar Distance Inches", mLidar.getDistanceCm());
        // ArmSubsystemTab.add("A Real Bar Distance", ArmConstants.getBarDistance(kArmLidarId));
        // ArmSubsystemTab.add("A Motor Velocity", mArmMotor.getEncoder().getVelocity());

        //SmartDashboard.putNumber("Raw Bar Distance", mLidar.getDistanceIn());
        SmartDashboard.putNumber("Bar Distance", getBarDistance());
        SmartDashboard.putNumber("Arm Motor Velocity", mArmMotor.getEncoder().getVelocity());
    }

    @Override
    public boolean checkSystem() {

        boolean isFunctional = true;
        double lidarDistance = mLidar.getDistanceIn();

        //Value should be lidar distance when arm is fully retractred
        if(lidarDistance < 0) {
            System.out.println("ERROR : Arm Lidar not functional or misaligned. Lidar distance = " + lidarDistance);
            isFunctional = false;
        }
          
        return isFunctional;
    }

    public void TESTSPEED(){
        setSpeed(-1.0);
    }

    private void setSpeed(double speed) {
        //System.out.println("Motor Controller speed" + speed);
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
