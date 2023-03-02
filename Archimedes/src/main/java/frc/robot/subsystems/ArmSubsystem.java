package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ArmConstants;
import frc.robot.enums.ArmState;
import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Ports.ArmPorts.*;

public class ArmSubsystem extends AftershockSubsystem {

    private static ArmSubsystem mInstance;
    private CANSparkMax mArmMotor;

    private final Lidar mLidar;
    private ProfiledPIDController mProfileController; //Add final back
    private final TrapezoidProfile.Constraints mConstraints;

    private ArmState mCurrentState;
    private ArmState mDesiredState;

    ShuffleboardTab ArmSubsystemTab = Shuffleboard.getTab("Arm Subsystem");
    GenericEntry P = ArmSubsystemTab.add("Arm P", 0).getEntry();
    GenericEntry I = ArmSubsystemTab.add("Arm I", 0).getEntry();
    GenericEntry D = ArmSubsystemTab.add("Arm D", 0).getEntry();

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
        mProfileController = new ProfiledPIDController(P.getDouble(0), I.getDouble(0), D.getDouble(0), mConstraints);
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

    public void jogArm(boolean isOut) {
        setSpeed(isOut ? kJogSpeed : -kJogSpeed);
    }

    public ArmState getState() {
        return mCurrentState;
    }

    public void setDesiredState(ArmState desiredState) {
        mDesiredState = desiredState;
    }

    @Override
    public void outputTelemetry() {

        ArmSubsystemTab.add("Raw Lidar Distance Inches", mLidar.getDistanceCm());
        ArmSubsystemTab.add("Real Bar Distance", ArmConstants.getBarDistance(kArmLidarId));
        ArmSubsystemTab.add("Motor Velocity", mArmMotor.getEncoder().getVelocity());

    }

    private void setSpeed(double speed) {
        mArmMotor.set(speed);
    }

    public static synchronized ArmSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ArmSubsystem();
        }    
        return mInstance;
    }
}
