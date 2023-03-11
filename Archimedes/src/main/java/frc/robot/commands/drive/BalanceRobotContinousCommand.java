package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceRobotContinousCommand extends CommandBase {

    private PID mPID; 
    private DriveSubsystem mDrive;

    public BalanceRobotContinousCommand(DriveSubsystem drive) {
        mDrive = drive;
        mPID = new PID();
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mPID.start(DriveConstants.kBalanceRobotGains);

    }

    @Override
    public void execute() {

        double robotPitch = mDrive.getPitch();
        double robotRoll = mDrive.getRoll();
        //double tiltVector = Math.sqrt(Math.pow(robotPitch, 2) + Math.pow(robotRoll, 2));

        double speed = mPID.update(robotPitch, 0);
        mDrive.drive(new ChassisSpeeds(speed, 0, 0));

    }
    
    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
    
}