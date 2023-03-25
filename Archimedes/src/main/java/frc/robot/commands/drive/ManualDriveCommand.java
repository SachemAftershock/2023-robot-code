package frc.robot.commands.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.enums.ArmState;
import frc.robot.enums.ElevatorState;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.DriveConstants.kDriveSpeedScaleFactor;
import static frc.robot.Constants.DriveConstants.kDriveSpeedFastScaleFactor;

public class ManualDriveCommand extends CommandBase {
    private final DriveSubsystem mDrivetrainSubsystem;
    private final Supplier<ArmState> mArmStateSupplier;
    private final Supplier<Boolean> mArmStowedEnoughSupplier;
    private final Supplier<Double> mElevatorHeightSupplier;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private final Supplier<Boolean> mIsSlowModeSupplier;
    private SlewRateLimiter mRateLimiter;

    public ManualDriveCommand(
        DriveSubsystem drivetrainSubsystem, Supplier<ArmState> armStateSupplier,
        Supplier<Boolean> armStowedEnoughSupplier, Supplier<Double> elevatorStateSupplier,
        DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier,
        Supplier<Boolean> isSlowMode
    ) {
        this.mDrivetrainSubsystem = drivetrainSubsystem;
        mArmStateSupplier = armStateSupplier;
        mArmStowedEnoughSupplier = armStowedEnoughSupplier;
        mElevatorHeightSupplier = elevatorStateSupplier;

        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        mIsSlowModeSupplier = isSlowMode;

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

        if (mIsSlowModeSupplier.get()) {
            xSpeed *= kDriveSpeedScaleFactor;
            ySpeed *= kDriveSpeedScaleFactor;
            rotSpeed *= kDriveSpeedScaleFactor;
        }
        
        // if (mArmStowedEnoughSupplier.get()) {
        // xSpeed *= kDriveSpeedFastScaleFactor;
        // ySpeed *= kDriveSpeedFastScaleFactor;
        // }
        // else if ((mArmStateSupplier.get() != ArmState.eStowEmpty
        // || mElevatorStateSupplier.get() != ElevatorState.eStowEmpty)) {

        // if (mArmStateSupplier.get() != null && mElevatorStateSupplier.get() != null)
        // {
        // xSpeed *= kDriveSpeedScaleFactor;
        // ySpeed *= kDriveSpeedScaleFactor;
        // }
        // }

        if(mElevatorHeightSupplier.get() > ElevatorState.eMid.getHeight()) {
            xSpeed *= kDriveSpeedScaleFactor;
            ySpeed *= kDriveSpeedScaleFactor;
            //rotSpeed *= kDriveSpeedScaleFactor;
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
