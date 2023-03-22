package frc.robot.commands.arm;

import frc.lib.PID;
import frc.robot.Constants.ArmConstants;
import frc.robot.enums.ArmState;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AttachHookCommand extends CommandBase {
    private ArmState mDesiredState;
    private ArmSubsystem mArmSubsystem;
    private PID mPID; 
    private double mCurrent;
    private double mSetpoint;

    public AttachHookCommand(ArmSubsystem armSubsystem) {
        mArmSubsystem = armSubsystem;
        mPID = new PID();
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        mPID.start(ArmConstants.kHookMotorGains);
        mCurrent = mArmSubsystem.getHookPosition();
        mSetpoint = ArmConstants.kHoookAttachedPosition;
    }

    @Override
    public void execute() {
        double speed = mPID.update(mCurrent, mSetpoint);
        mArmSubsystem.setHookSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(mPID.getError()) < ArmConstants.kHookEpsilon;
    }

    @Override
    public void end(boolean interrupted) {
        mArmSubsystem.stop();
    }   
}
