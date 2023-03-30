package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.BalanceConstants.kBalanceKillDelta;

public class BalanceRobotCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mLastPitch;
    private boolean mIsFinished;

    private double[] pitches;
    private int index;

    public enum BalanceSteps {
        eStateOne, eStateTwo
    }

    private BalanceSteps mBalanceSteps;

    public BalanceRobotCommand(DriveSubsystem drive) {
        mDrive = drive;
        mIsFinished = false;
        mLastPitch = 0;
        pitches = new double[20];
        index = 0;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        System.out.println("---------- STARTING BALANCE COMMAND ----------");
        mDrive.drive(new ChassisSpeeds(-20.0, 0.0, 0));
        mBalanceSteps = BalanceSteps.eStateOne;
    }

    @Override
    public void execute() {

        pitches[index++]  = -mDrive.getPitch();
        if (index < 20) return; 

        double delta = pitches[19] - pitches[0];

        // If our current pitch is less than our last pitch by a certain threshold,
        // we know we're beginning to become level, so kill the motors
        // and let momentum do the rest
        if (delta < kBalanceKillDelta) {
            System.out.println("ERROR: killing bot ");
            mIsFinished = true;
        }
        
        pitches = shiftArray(pitches);
        index = 19;
    }

    public double[] shiftArray(double[] arr) {
        double[] out = new double[arr.length];
        for (int i = 1; i < arr.length; i++) {
            out[i - 1] = arr[i];
        }
        return out;
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