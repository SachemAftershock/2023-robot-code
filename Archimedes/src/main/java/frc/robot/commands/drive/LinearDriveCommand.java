package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.CardinalDirection;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class LinearDriveCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mLinearSetpoint;
    private PID mPid;
    private double mCurrentPose;
    private CardinalDirection mDirection;
    private SlewRateLimiter mRateLimiter;

    // Field Relative : Y direction is horizontal, X direction is downfield

    //Setup position is the robot's intake is facing the grid
    //Positive X makes the robot drive forward (drives into the grid from setup position)
    //Negative X makes the robot drive backwards (drives into AWAY from the grid from setup position)
    //Negative Y (from drivers POV) makes it drive towards the left 
    //Positive Y (from drivers POV) makes it drive towards the right

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

        mRateLimiter = new SlewRateLimiter(0.5);
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

        double speed = mPid.update(mCurrentPose, mLinearSetpoint) * DriveConstants.kAutoMaxVelocityMetersPerSecond * 10;

        // speed = mRateLimiter.calculate(speed);
        System.out.println(
            "Current --> " + mCurrentPose + " Setpoint --> " + mLinearSetpoint + " Error --> "
                + Math.abs(mPid.getError())
        );
        //speed = speed * 0.4;

        if (direction) {
            mDrive.drive(new ChassisSpeeds(0, speed, 0));
            // System.out.println("Wrong");
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