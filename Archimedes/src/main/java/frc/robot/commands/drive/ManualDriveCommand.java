package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.enums.ArmState;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.DriveConstants.kDriveSpeedScaleFactor;

public class ManualDriveCommand extends CommandBase {
    private final DriveSubsystem mDrivetrainSubsystem;
    private final Supplier<ArmState> mArmStateSupplier;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public ManualDriveCommand(
        DriveSubsystem drivetrainSubsystem, Supplier<ArmState> armStateSupplier, DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier
    ) {
        this.mDrivetrainSubsystem = drivetrainSubsystem;
        mArmStateSupplier = armStateSupplier;

        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        double xSpeed = m_translationXSupplier.getAsDouble();
        double ySpeed = m_translationYSupplier.getAsDouble();

        double rotSpeed = m_rotationSupplier.getAsDouble();

        if (mArmStateSupplier.get() != ArmState.eStow) {
            xSpeed *= kDriveSpeedScaleFactor;
            ySpeed *= kDriveSpeedScaleFactor;
        }

        mDrivetrainSubsystem
            .drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, mDrivetrainSubsystem.getGyroscopeRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}