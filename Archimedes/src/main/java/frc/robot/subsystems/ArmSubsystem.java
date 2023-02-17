package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.AftershockSubsystem;
import frc.robot.Constants.ArmConstants;

import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends AftershockSubsystem {

    private static ArmSubsystem mInstance;
    private CANSparkMax mArmMotor;
    private SparkMaxPIDController mPIDController;
    
    private final TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    private static double kDt;

    public ArmSubsystem() {
        super();

        mArmMotor = new CANSparkMax(kArmMotorID, MotorType.kBrushless);
        m_constraints =  new TrapezoidProfile.Constraints(1.75, 0.75);
        m_goal = new TrapezoidProfile.State();
        m_setpoint = new TrapezoidProfile.State();

        kDt = ArmConstants.kDt;

        mPIDController = mArmMotor.getPIDController();

        mPIDController.setP(kP);
        mPIDController.setI(kI);
        mPIDController.setD(kD);
        mPIDController.setIZone(kIz);

    }

    @Override
    public void initialize() {
        m_goal = new TrapezoidProfile.State(0.5, 0.5);
    }

    @Override
    public void periodic() {

        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(kDt);
    
        mPIDController.setReference(m_setpoint.position, CANSparkMax.ControlType.kPosition);
    }
    

    @Override
    public void outputTelemetry() {
        
    }

    public static synchronized ArmSubsystem getInstance() {
        if(mInstance != null) {
            mInstance = new ArmSubsystem();
        }
        return mInstance;        
    }
    
}



