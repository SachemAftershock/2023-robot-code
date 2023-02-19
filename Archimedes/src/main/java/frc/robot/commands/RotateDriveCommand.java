package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.PID;
import frc.lib.Util;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class RotateDriveCommand extends CommandBase {

    private DriveSubsystem mDrive;
    private double mAngularSetpoint;
    private PID mPid;
    
    //Field Relative Rotation, downfield is 0deg
    public RotateDriveCommand(DriveSubsystem drive, double setpoint) {
        mDrive = drive;
        mAngularSetpoint = setpoint;
        mPid = new PID();
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mPid.start(DriveConstants.kDriveAngularGains);
        System.out.println("RotateDriveCommand started " + Double.toString(mAngularSetpoint) + " degrees.");
    }

    @Override
    public void execute() {
        double currentAngle = Util.normalizeAngle(mDrive.getGyroscopeRotation().getDegrees());
		double rotationSpeed = mPid.updateRotation(currentAngle, mAngularSetpoint)
				* DriveConstants.kMaxAngularVelocityRadiansPerSecond * 0.5;
        
        mDrive.drive(new ChassisSpeeds(0, 0, rotationSpeed));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(mPid.getError()) < DriveConstants.kAutoRotateEpsilon;
    }
    @Override
    public void end(boolean interrupted) {
        mDrive.drive(new ChassisSpeeds());
    }
}