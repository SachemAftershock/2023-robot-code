package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.SlidingWindow;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.BalanceConstants.*;

public class BalanceRobotForceDampCommand extends CommandBase {
    private final double kMaxSpeed = 15.0;
    private final double kDampRate = 5.0;

    private final DriveSubsystem mDrive;

    private final SlidingWindow mWindow;
    private double mCurrentSpeedDamp = kMaxSpeed;

    private boolean mLastDecremented = false;

    private int iteration = 0;
    public BalanceRobotForceDampCommand(DriveSubsystem drive) {
        mDrive = drive;
        mWindow = new SlidingWindow(kWindowSize);
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.drive(new ChassisSpeeds(-kMaxSpeed, 0,0));
    }

    @Override
    public void execute() {
        mWindow.add(-mDrive.getPitch());
        if(!mWindow.windowReady()) return;

        if (mWindow.getDelta().doubleValue() < kBalanceKillDelta && !mLastDecremented) {
            mCurrentSpeedDamp -= kDampRate;
            mDrive.drive(new ChassisSpeeds(-mCurrentSpeedDamp, 0,0));
            mLastDecremented = true;
        }
        else if (mWindow.getDelta().doubleValue() > -kBalanceKillDelta) {
            mDrive.drive(new ChassisSpeeds(mCurrentSpeedDamp,0,0));
        }
        else {
            ++iteration;
            if (iteration > 100) {
                mLastDecremented = false;
                iteration = 0;
            }
        }

        System.out.println("Speed --> " + mCurrentSpeedDamp);
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
        mDrive.lockWheels();
    }

    @Override
    public boolean isFinished() {
        return mCurrentSpeedDamp <= 0;
    }
}
