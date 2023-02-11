import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.subsystems.ElevatorSubsystem;

public class CommandFactory{
    public static SequentialCommandGroup commandFactory(ElevatorPosition currentState, ElevatorPosition desiredState, ElevatorSubsystem elevatorSubsystem) {
    if(currentState.equals(ElevatorPosition.STOW)) {
        return new SequentialCommandGroup(
        new ElevatorCommand(elevatorSubsystem, desiredState)
        );
         
    }
}
    
}


