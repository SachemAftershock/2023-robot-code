package frc.robot.subsystems;

import frc.lib.AftershockSubsystem;
import frc.robot.Constants.ControllerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.IntakeCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorConstants;
public class IntakeSubsystem extends AftershockSubsystem 
{

    //TODO: test motor with this code, add "brushless" motor type to sparkmax object
    //TODO: add constants for motor speed for all 4 situations
    
    private CANSparkMax sparkMotorController = new CANSparkMax(MotorConstants.kCANSparkMaxID, MotorType.kBrushless); 
    public IntakeSubsystem(){
        super();
    }

    public void intakePeriodic(){
    }

    //Turn on the motor to input the cone
    //Button right trigger
    //clockwise to output

    /**
     * 
     * This sets the motor speed
     * 
     */
    public double getCurrent()
    {
        //return sparkMotorController.getOutputCurrent();
        return sparkMotorController.getBusVoltage();
    }
    private void setSpeed(double speed){
        sparkMotorController.set(speed);
    }
    public void ingestCone(){
        setSpeed(-(IntakeConstants.kIngestConeSpeed));
    }
    //Turn on the motor to output the cone
    //Button left trigger
    //counterclockwise to intake
    public void outputCone(){ 
        setSpeed(IntakeConstants.kIngestConeSpeed);
        //System.out.println(sparkMotorController.get());
    }
    //Turn on the motor to input the cube
    //Button right bumper
    //counterclockwise to input
    public void ingestCube(){
        setSpeed(IntakeConstants.kIngestCubeSpeed);
        //System.out.println(sparkMotorController.get());
    }
    //Turn on the motor to output the cube
    //Button left bumper
    //clockwise to output
    public void outputCube(){
        setSpeed(-(IntakeConstants.kIngestCubeSpeed));
       // System.out.println(sparkMotorController.get());
    }

    public void stopIntakeMotor() {
        sparkMotorController.set(0);
       // System.out.println(sparkMotorController.get());
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }

//right trigger, left trigger - cones
//right bumper left bumper - cubes
//right for consume, left for expel
    }
