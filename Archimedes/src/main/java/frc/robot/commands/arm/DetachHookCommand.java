package frc.robot.commands.arm;

import frc.robot.enums.ArmState;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DetachHookCommand extends CommandBase {
    private ArmState mDesiredState;
    private ArmSubsystem mArmSubsystem;

    public DetachHookCommand(ArmSubsystem armSubsystem) {
        mArmSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
       
    }

    // @Override
    // public boolean isFinished() {
    //    
    // }
}

