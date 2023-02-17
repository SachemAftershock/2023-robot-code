package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.enums.ArmState;
import frc.robot.enums.SuperState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CommandFactory {
    
    public static SequentialCommandGroup commandFactory(
        SuperState desiredState,
        ElevatorSubsystem elevatorSubsystem,
        ArmSubsystem armSubsystem
    ) {
        if (armSubsystem.getState() == ArmState.eStow) {
            return new SequentialCommandGroup(
              new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem)
              //set arm to desired state
            );
        } 
        else {
            return new SequentialCommandGroup(
                //return arm to stow point
                new SetElevatorStateCommand(desiredState.getElevatorState(), elevatorSubsystem)
                //set arm to desired state
            );
        }
    }
}
