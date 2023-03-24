package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmStateCommand;
import frc.robot.commands.elevator.SetElevatorStateCommand;
import frc.robot.enums.ArmState;
import frc.robot.enums.ElevatorState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class FloorPickupCommand extends SequentialCommandGroup {
    
    public FloorPickupCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
        addCommands(
            new SetElevatorStateCommand(ElevatorState.eClearBumper, elevator),
            new SetArmStateCommand(ArmState.eLow, arm),
            new SetElevatorStateCommand(ElevatorState.eLow, elevator)
        );
    }

}
