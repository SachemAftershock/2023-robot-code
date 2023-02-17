package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.enums.ElevatorPosition;
import frc.robot.subsystems.ElevatorSubsystem;

public class CommandFactory extends SequentialCommandGroup{
    public static SequentialCommandGroup commandFactory(ElevatorPosition currentState, ElevatorPosition desiredState, ElevatorSubsystem elevatorSubsystem) {
    
        if(currentState.equals(ElevatorPosition.STOW)) {
        return new SequentialCommandGroup(new ElevatorCommand(elevatorSubsystem, desiredState));
    }
    else {
        
        return new SequentialCommandGroup();
    }

}
}
