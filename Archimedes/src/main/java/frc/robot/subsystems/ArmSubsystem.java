package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.lib.AftershockDifferentialDrive;
import frc.lib.AftershockSubsystem;
import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends AftershockSubsystem {

    private static ArmSubsystem mInstance;
    private CANSparkMax mArmMotor;
    private TrapezoidProfile mMotionProfile;
    private Constraints mConstraints;
    private SparkMaxPIDController mPIDController;
    private RelativeEncoder mEncoder;

    public ArmSubsystem() {
        super();
        mArmMotor = new CANSparkMax(kArmMotorID, MotorType.kBrushless);
        // mConstraints = new TrapezoidProfile.Constraints(ArmConstants.kMaxVelocityRadPerSecond, 
        //                                                 ArmConstants.kMaxAccelerationRadPerSecSquared);
        mPIDController = mArmMotor.getPIDController();
        mEncoder = mArmMotor.getEncoder();

        mPIDController.setP(kP);
        mPIDController.setI(kI);
        mPIDController.setD(kD);
        mPIDController.setIZone(kIz);
        mPIDController.setFF(kFF);
        mPIDController.setOutputRange(kMinOutput, kMaxOutput);

        mPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        mPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        mPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        mPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub

       

        
    }

    @Override
    public void periodic() {

    }
    
    public void useState(TrapezoidProfile.State setpoint) {

    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }

    public static synchronized ArmSubsystem getInstance() {
        if(mInstance != null) {
            mInstance = new ArmSubsystem();
        }
        return mInstance;        
    }
    
}



