package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

public class ManualDriveCommand extends CommandBase {
    private final DriveSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private final SlewRateLimiter mRateLimiter;

    public ManualDriveCommand(DriveSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        mRateLimiter = new SlewRateLimiter(DriveConstants.kDriveRateLimit);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        
        double xSupplier = mRateLimiter.calculate(m_translationXSupplier.getAsDouble());
        double ySupplier = mRateLimiter.calculate(m_translationYSupplier.getAsDouble());

        //double rotationSupplier = mRateLimiter.calculate(m_rotationSupplier.getAsDouble());
        //If rate limiting needed for rotations
        double rotationSupplier = mRateLimiter.calculate(m_rotationSupplier.getAsDouble());

        var convertedVals = m_drivetrainSubsystem.runAntiTiltControl(xSupplier, ySupplier);
        
        m_drivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(convertedVals[1], convertedVals[0], rotationSupplier, m_drivetrainSubsystem.getGyroscopeRotation())
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
