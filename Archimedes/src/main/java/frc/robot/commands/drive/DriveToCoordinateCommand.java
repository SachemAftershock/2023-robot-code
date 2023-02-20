package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToCoordinateCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mXCoordSetpoint;
    private double mYCoordSetpoint;

    private Pose2d mSetpoint;

    private PID mPIDX;
    private PID mPIDY;

    private boolean mIsFinished = false;

    public DriveToCoordinateCommand(Pose2d setpoint, DriveSubsystem drive) {
        mDrive = drive;
        mSetpoint = setpoint;

        mPIDX = new PID();
        mPIDY = new PID();
    }

    @Override
    public void initialize() {

        System.out.println("Drive to Slot command starting ");

        // Arhum did this to pass in the april tag as the setpoint and be a little off
        // of it
        // TODO: Consider moving this logic elsewhere, so the command works as intended
        mXCoordSetpoint = mSetpoint.getX() - 0.5;
        mYCoordSetpoint = mSetpoint.getY() - 0.5;

        mPIDX.start(DriveConstants.kDriveLinearGains);
        mPIDY.start(DriveConstants.kDriveLinearGains);

        if ((Math.abs(mDrive.getPose().getX())) < DriveConstants.kMinimumDistanceForAutoDrive
            && Math.abs(mDrive.getPose().getY()) < DriveConstants.kMinimumDistanceForAutoDrive) {
            mIsFinished = true;
        }

    }

    @Override
    public void execute() {
        if (mIsFinished) return;

        double xCurrent = mDrive.getPose().getX();
        double yCurrent = mDrive.getPose().getY();

        double xSpeed = mPIDX.update(xCurrent, mXCoordSetpoint);
        double ySpeed = mPIDY.update(yCurrent, mYCoordSetpoint);

        if (Math.abs(mPIDX.getError()) < DriveConstants.kDriveToTargetEpsilon) {
            xSpeed = 0.0;
        }

        if (Math.abs(mPIDY.getError()) < DriveConstants.kDriveToTargetEpsilon) {
            ySpeed = 0.0;
        }

        if (Math.abs(mPIDX.getError()) < DriveConstants.kDriveToTargetEpsilon
            && Math.abs(mPIDY.getError()) < DriveConstants.kDriveToTargetEpsilon) {
            return;
        }

        // System.out.println("X --> " + xCurrent + " Y--> " + yCurrent + " X speed -->
        // " + xSpeed + " Y speed --> " + ySpeed);
        mDrive.drive(new ChassisSpeeds(xSpeed, ySpeed, 0));
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
