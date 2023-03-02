package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class DelayCommand extends CommandBase {

    private double mSecondsToDelay;
    private Timer mTimer;

    public DelayCommand(double theSecondsToDelay) {
        mSecondsToDelay = theSecondsToDelay;
        mTimer = new Timer();
        addRequirements();
    }   

    @Override
    public void initialize() {
        System.out.println("DelayCommand started " + Double.toString(mSecondsToDelay) + " seconds.");
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        
        if (mTimer.hasElapsed(mSecondsToDelay)) {
            System.out.println("DelayCommand completed " + Double.toString(mSecondsToDelay) + " seconds.");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        mTimer.stop();
    }
}