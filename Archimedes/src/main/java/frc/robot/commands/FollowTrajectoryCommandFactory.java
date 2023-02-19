package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.*;

public class FollowTrajectoryCommandFactory {
    
    public static SwerveControllerCommand generateCommand(DriveSubsystem driveSubsystem, Trajectory trajectory) {
        final double kP = kDriveAngularGains[0];
        final double kI = kDriveAngularGains[1];
        final double kD = kDriveAngularGains[2];

        final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            kP, kI, kD, 
            new TrapezoidProfile.Constraints(
                kMaxAngularVelocityRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared
            )
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        return (
            new SwerveControllerCommand(
                trajectory, 
                driveSubsystem::getPose,
                driveSubsystem.getKinematics(),
                new PIDController(kPX, 0, 0), new PIDController(kPY, 0, 0), thetaController, 
                driveSubsystem::drive,
                driveSubsystem
            )
        );
    }
}
