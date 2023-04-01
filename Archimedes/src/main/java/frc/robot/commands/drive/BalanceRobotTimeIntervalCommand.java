package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.SlidingWindow;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.BalanceConstants.*;

public class BalanceRobotTimeIntervalCommand extends CommandBase {

    private final DriveSubsystem mDrive;

    private double mTiltBackTime;
    private double mTiltForwardTime;

    private State mState;

    private final SlidingWindow mWindow;

    enum State {
        SCALE_CHARGE_STATION,
        TILT_BACK,
        TILT_FORWARD,
        FIND_MIDDLE,
        FINISH
    }

    public BalanceRobotTimeIntervalCommand(DriveSubsystem drive) {
        mDrive = drive;
        mTiltBackTime = 0;
        mTiltForwardTime = 0;
        mState = State.SCALE_CHARGE_STATION;
        mWindow = new SlidingWindow(kWindowSize, kBalanceKillDelta);
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.drive(new ChassisSpeeds(kSpeedAttack, 0, 0));
    }

    @Override
    public void execute() {
        mWindow.add(-mDrive.getPitch());
        if (!mWindow.windowReady()) return;

        System.out.println("State --> " + mState);
        final double tipAngle = 10.0;
        switch (mState) {
            case SCALE_CHARGE_STATION:
                if (mWindow.getDelta().doubleValue() < -tipAngle) {
                    mState = State.TILT_BACK;
                    mDrive.drive(new ChassisSpeeds(kSecondSpeed + 5, 0, 0));
                }
                break;
            case TILT_BACK:
                if (mWindow.getDelta().doubleValue() > tipAngle) {
                    mState = State.TILT_FORWARD;
                    mTiltBackTime = Timer.getFPGATimestamp();
                    mDrive.drive(new ChassisSpeeds(-(kSecondSpeed + 5), 0, 0));
                }
                break;
            case TILT_FORWARD:
                if (mWindow.getDelta().doubleValue() < -tipAngle - 0.5) {
                    mTiltForwardTime = Timer.getFPGATimestamp();
                    mDrive.drive(new ChassisSpeeds(kSecondSpeed + 5, 0, 0));
                    mState = State.FIND_MIDDLE;
                }
                break;
            case FIND_MIDDLE:
                if (Timer.getFPGATimestamp() >= ((mTiltForwardTime - mTiltBackTime) * 0.5) + mTiltForwardTime) {
                    mState = State.FINISH;
                }
        };
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
        mDrive.lockWheels();
    }

    @Override
    public boolean isFinished() {
        return mState == State.FINISH;
    }
}
