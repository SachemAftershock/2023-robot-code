package frc.robot.commands.arm;

import frc.lib.PID;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.ArmConstants.*;

public class DetachHookCommand extends CommandBase {
    private ArmSubsystem mArmSubsystem;
    private PID mPID;

    public DetachHookCommand(ArmSubsystem armSubsystem) {
        mArmSubsystem = armSubsystem;
        mPID = new PID();
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("detatch");
        mPID.start(kHookMotorGains);
    }

    @Override
    public void execute() {
        double current = mArmSubsystem.getHookPosition();
        double setpoint = kHoookDetachedPosition;

        double speed = mPID.update(current, setpoint);
        mArmSubsystem.setHookSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(mPID.getError()) < kHookEpsilon;
    }

    @Override
    public void end(boolean interrupted) {
        mArmSubsystem.stopHook();
    }
}
