package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.enums.ArmState;
import frc.robot.enums.ElevatorState;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.DriveConstants.*;

public class ManualDriveCommand extends CommandBase {
    private final DriveSubsystem mDrivetrainSubsystem;
    private final Supplier<Boolean> mArmStowedEnoughSupplier;
    private final Supplier<ElevatorState> mElevatorStateSupplier;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private SlewRateLimiter mRateLimiter;

    public ManualDriveCommand(
        DriveSubsystem drivetrainSubsystem, Supplier<Boolean> armStowedEnoughSupplier,
        Supplier<ElevatorState> elevatorStateSupplier, DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier
    ) {
        this.mDrivetrainSubsystem = drivetrainSubsystem;
        mArmStowedEnoughSupplier = armStowedEnoughSupplier;
        mElevatorStateSupplier = elevatorStateSupplier;

        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        mRateLimiter = new SlewRateLimiter(0.5);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        double xSpeed = m_translationXSupplier.getAsDouble();
        double ySpeed = m_translationYSupplier.getAsDouble();

        double rotSpeed = m_rotationSupplier.getAsDouble();

        if (!mArmStowedEnoughSupplier.get()) {
            xSpeed *= kArmOutSpeedScaleFactor;
            ySpeed *= kArmOutSpeedScaleFactor;
        }
        else if (mElevatorStateSupplier.get() != ElevatorState.eStowEmpty) {
            xSpeed *= kArmStowedEnoughScaleFactor;
            ySpeed *= kArmOutSpeedScaleFactor;
        }

        mDrivetrainSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, mDrivetrainSubsystem.getGyroscopeRotation())
        );
    }

    @Override
    public void end(boolean interrupted) {
        mDrivetrainSubsystem.drive(new ChassisSpeeds());
    }
}
