package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.BalanceConstants.kBalanceKillDelta;

public class HoldPositionCommand extends InstantCommand {

    private DriveSubsystem mDrive;
    private double mLastPitch;
    private boolean mIsFinished;

    public HoldPositionCommand(DriveSubsystem drive) {
        mDrive = drive;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {

        Rotation2d lockedRotation = new Rotation2d(45);

        SwerveModuleState[] states = {
            new SwerveModuleState(0, lockedRotation),
            new SwerveModuleState(0, lockedRotation),
            new SwerveModuleState(0, lockedRotation),
            new SwerveModuleState(0, lockedRotation),
        };
        mDrive.drive(states);
    }

    @Override
    public void execute() {
       
    }

    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished() {
        return mIsFinished;
    }

}