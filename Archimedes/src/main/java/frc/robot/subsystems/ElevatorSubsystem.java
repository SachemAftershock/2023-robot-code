package frc.robot.subsystems;
import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;
import frc.lib.PID;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PIDvalues;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


public class ElevatorSubsystem extends AftershockSubsystem {
    
    private Lidar m_lidar = new Lidar(ControllerConstants.kPrimaryControllerPort);   
    private PID mPID = new PID();
    private double elevatorDistance;     
    private CANSparkMax mMotor;
    private Joystick ButtonBox;
    private static ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();

    private double setpoint;

    public void ElevatorSubsystem(XboxController mXboxController) {

        setpoint = 0.0;
        mMotor = new CANSparkMax(0, MotorType.kBrushless);
    }
    public void ElevatorPeriodic()
    {
        
        elevatorDistance = m_lidar.getDistanceIn();//lidar distance in inches
        
    }
    public void ElevatorTerminate(){
        //if(elevatorDistance==){
        //   elevatorDistance=
        //}
    }
    public void initialize() {
        setpoint = 0.0;
        // TODO Auto-generated method stub
        //mPID.start(PIDvalues.kPIDvalue);
    }   
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }

    public void startElevatorPID(double setpoint) {

        this.setpoint = setpoint;
        mPID.start(PIDvalues.kPIDvalue);

    }

    public void runPID() {
        double current = m_lidar.getDistanceIn();
        double speed = mPID.update(current, setpoint);
        setElevatorSpeed(speed);
    }

    public boolean isFinished() {
        return  mPID.getError() < PIDvalues.kElevatorEpsilon;
    }

    public void end(){
        mMotor.set(0);
    }

    public void setElevatorSpeed(double speed){
        mMotor.set(speed);
    }
    public static ElevatorSubsystem getInstance() {
        return mElevatorSubsystem;
    }
    /**public void setState()
    {
    
    }**/
    
}
