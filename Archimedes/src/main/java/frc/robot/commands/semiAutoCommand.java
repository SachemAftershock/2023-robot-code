package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drive.FollowTrajectoryCommandFactory;

public class semiAutoCommand extends CommandBase {
    private TrajectoryConfig config = new TrajectoryConfig(
        DriveConstants.kAutoMaxVelocityMetersPerSecond * 0.3, DriveConstants.kMaxAccelerationMetersPerSecondSquared
    );
    // private double distanceFromTag = 0.5588; // distance between scoring points
    private double channelMidpoint = 0.770763; // the midpoint of the channel
    private double distanceFromBuffer = 0.41 + .1875; // The distance between the apriltag and the channel, measured from the cone
                                                      // scoring area + a buffer number
    private DriveSubsystem mDrive;

    public semiAutoCommand(Pose2d startingPoint, double xPose, double yPose, DriveSubsystem drive, int right) {
        mDrive = drive;
        // double finalYPose = xPose;
        // // If right is one, then the robot will shift to the right of the selected
        // // apriltag
        // // If right is -one, then the robot will shift to the left of the selected
        // // apriltag
        // // If right is neither, then the robot will go to the apriltag
        // if (right == 1) {
        // finalYPose += distanceFromTag;
        // }
        // else if (right == -1) {
        // finalYPose -= distanceFromTag;
        // }

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            startingPoint, List.of(
                new Translation2d(xPose - channelMidpoint - distanceFromBuffer, yPose)

            ), new Pose2d(xPose - channelMidpoint - distanceFromBuffer, yPose, new Rotation2d(Math.PI)), config
        );
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(), List.of(
                new Translation2d(xPose - distanceFromBuffer, yPose)

            ), new Pose2d(xPose - distanceFromBuffer, yPose, new Rotation2d()), config
        );
        new SequentialCommandGroup(
            FollowTrajectoryCommandFactory.generateCommand(mDrive, trajectory),
            FollowTrajectoryCommandFactory.generateCommand(mDrive, trajectory2)
        );
    }
}
