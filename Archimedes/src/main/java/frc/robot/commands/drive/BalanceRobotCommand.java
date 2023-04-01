package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.lib.SlidingWindow;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.BalanceConstants.*;

public class BalanceRobotCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mLastPitch;
    private boolean mIsFinished;
    private boolean mIsPID; 

    private PID mPID; 

    private SlidingWindow mWindow;

    private double mStartStableTime = 0;

    private double mCommandStartTime = 0;

    int in = 0;

    private double mFirstTipBackTime = 0;

    public BalanceRobotCommand(DriveSubsystem drive) {
        mDrive = drive;
        mIsFinished = false;
        mLastPitch = 0;
   
        mPID = new PID();

        mWindow = new SlidingWindow(kWindowSize, kBalanceKillDelta);

        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        System.out.println("---------- STARTING BALANCE COMMAND ----------");
        mDrive.drive(new ChassisSpeeds(kSpeedAttack, 0.0, 0));
        mPID.start(kBalancePIDGains);
        mCommandStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

        if (Timer.getFPGATimestamp() - mCommandStartTime >= 11) {
            mIsFinished = true;
            return;
        }
        //System.out.println("Pitch --> " + -mDrive.getPitch());

        if (mDrive.getPitch() <= -12) {

        }

        if (mIsPID) {
            //System.out.println("---------- STARTING PID ----------");
            double speed = -mPID.update(mDrive.getPitch(), 0.0) * 4.25;
            System.out.println(speed);
            mDrive.drive(new ChassisSpeeds(speed, 0.0, 0.0));

            if (Math.abs(mPID.getError()) < kBalanceRobotEpsilon - 0.5) {
                System.out.println("ERROR: ACTUALLY KILLING ROBOT");
                //mIsFinished = true;

                ++in;
                
                if (in > 2) {
                    mIsFinished = true;
                }

                // if (mStartStableTime != 0 && Timer.getFPGATimestamp() - mStartStableTime >= 500) {
                //     mIsFinished = true;
                // }


                // if (mStartStableTime == 0) {
                //     mStartStableTime = Timer.getFPGATimestamp();
                // }
            } else {
                mStartStableTime = 0;
            }

            return;
        }

        mWindow.add(-mDrive.getPitch());
        if (!mWindow.windowReady()) return;

        if (mWindow.getDelta().doubleValue() < 0) {
            mDrive.drive(new ChassisSpeeds(kSecondSpeed, 0.0, 0.0));
        }

        System.out.println("Delta --> " + mWindow.getDelta());

        if (mWindow.checkDeltaLess()) {
            System.out.println("ERROR: JUST KILLING ROBOT");
            mIsPID = true; 
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
        mDrive.lockWheels();
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

}