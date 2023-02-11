package frc.robot.commands;
import frc.lib.AftershockSubsystem;
import frc.lib.Lidar;
import frc.lib.PID;
import frc.robot.Constants.PortConstants;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.PIDvalues;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ElevatorCommand extends InstantCommand{
    private static ElevatorPosition mdesiredState;
    private ElevatorSubsystem mElevatorSubsystem;
    private final Lidar mLidarElevator = new Lidar(1); 


    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, ElevatorPosition desiredState ) 
    {        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements();
        mdesiredState = desiredState;
        mElevatorSubsystem = elevatorSubsystem;

    }

      // Called when the command is initially scheduled.
    @Override
    public void initialize() {

      /**double distance = mCurrentState.getDistance();
      mElevatorSubsystem.startElevatorPID(distance);*/
      
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      mElevatorSubsystem.runPID();
    }

    @Override
    public void end(boolean interrupted) {
      mElevatorSubsystem.end();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if(mElevatorSubsystem.isFinished()) {
        return true;
      } 
      return false;
    }
}


