package frc.robot.commands.arm;

import frc.robot.enums.ArmState;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmStateCommand extends CommandBase {
    private ArmState mDesiredState;
    private ArmSubsystem mArmSubsystem;

    public SetArmStateCommand(ArmState desiredState, ArmSubsystem armSubsystem) {
        mArmSubsystem = armSubsystem;
        mDesiredState = desiredState;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // System.out.println("Arm state set to " + mDesiredState);
        mArmSubsystem.setDesiredState(mDesiredState); // if broken move to execute
    }

    @Override
    public void end(boolean interrupted) {
        mArmSubsystem.stop();
        if (interrupted) mArmSubsystem.overrideCurrentState();
    }

    @Override
    public boolean isFinished() {
        return mArmSubsystem.getState() == mDesiredState;
    }
}
