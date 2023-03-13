package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class LinearDriveCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mLinearSetpoint;
    private PID mPid;
    private double mCurrentPose;
    private CardinalDirection mDirection;

    // Field Relative : Y direction is horizontal, X direction is downfield

    public LinearDriveCommand(DriveSubsystem drive, double setpoint, CardinalDirection direction) {
        mDrive = drive;
        mLinearSetpoint = setpoint;
        mDirection = direction;
        mPid = new PID();
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mCurrentPose = 0.0;

        if (mDirection == CardinalDirection.eY) {
            mLinearSetpoint += mDrive.getPose().getY();
        }
        else {
            mLinearSetpoint += mDrive.getPose().getX();
        }

        mPid.start(DriveConstants.kDriveLinearGains); // TODO: tune pid values
        System.out.println(
            "Linear Drive Command started : Current Pose --> " + mDrive.getPose() + " Setpoint " + mLinearSetpoint
        );
    }

    @Override
    public void execute() {

        /**
         * Y direction is true X direction is false
         */
        boolean direction = false;

        if (mDirection == CardinalDirection.eY) {
            mCurrentPose = mDrive.getPose().getY();
            direction = true;
        }
        else {
            mCurrentPose = mDrive.getPose().getX();
            direction = false;
        }

        double speed = mPid.update(mCurrentPose, mLinearSetpoint) * DriveConstants.kMaxVelocityMetersPerSecond;

        if (direction) {
            mDrive.drive(new ChassisSpeeds(0, speed, 0));
        }
        else {
            mDrive.drive(new ChassisSpeeds(speed, 0, 0));
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(mPid.getError()) < DriveConstants.kLinearDriveEpsilon;
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
    }
}