package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.BalanceConstants.kBalanceKillDelta;

public class BalanceRobotCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mLastPitch;
    private boolean mIsFinished;

    public BalanceRobotCommand(DriveSubsystem drive) {
        mDrive = drive;
        mIsFinished = false;
        mLastPitch = 0;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.drive(new ChassisSpeeds(0, -1d, 0));
    }

    @Override
    public void execute() {
        double robotPitch = mDrive.getPitch();

        // If our current pitch is less than our last pitch by a certain threshold,
        // we know we're beginning to become level, so kill the motors
        // and let momentum do the rest
        if (robotPitch - mLastPitch < kBalanceKillDelta) mIsFinished = true;
        mLastPitch = robotPitch;
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

}