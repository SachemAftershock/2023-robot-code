package frc.robot.auto.semiauto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.drive.DriveToWaypointCommand;
import frc.robot.commands.drive.SetWaypointCommand;

public class Station1 extends SequentialCommandGroup {

    private final DriveSubsystem mDrive;

    TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.kAutoMaxVelocityMetersPerSecond * 0.3, DriveConstants.kMaxAccelerationMetersPerSecondSquared
    );

    Pose2d startingPose;
    Pose2d endPose;

    double endX;
    double endY;

    public Station1(DriveSubsystem drive, int stationNum) {
        mDrive = drive;
        if (DriverStation.getAlliance() == Alliance.Red) {
            endX = DriveConstants.kBlueGridXCoordinate;
            endY = DriveConstants.kGridYCoordinates[8 - stationNum];
        }
        else if (DriverStation.getAlliance() == Alliance.Blue) {
            endX = DriveConstants.kRedGridXCoordinate;
            endY = DriveConstants.kGridYCoordinates[stationNum];
        }
        startingPose = mDrive.getPose();
        endPose = new Pose2d(1.9, .41, new Rotation2d(Math.PI));
        addCommands(new SetWaypointCommand(endPose, drive));
    }

}
