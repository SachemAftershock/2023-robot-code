package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

public class SetWaypointCommand extends InstantCommand {

    private final Pose2d mWaypoint;
    private final DriveSubsystem mDrive;

    public SetWaypointCommand(Pose2d waypoint, DriveSubsystem driveSubsystem) {
        mDrive = driveSubsystem;
        mWaypoint = waypoint;
    }

    @Override
    public void execute() {
        mDrive.setWaypoint(mWaypoint);
    }

}
