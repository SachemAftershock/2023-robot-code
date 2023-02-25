package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ButtonBoxPublisher;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.subsystems.DriveSubsystem;

public class SetWaypointCommand extends InstantCommand {

    private final Pose2d mWaypoint;
    private final DriveSubsystem mDrive;
    
    private LedPosition mLedPosition;

    public SetWaypointCommand(Pose2d waypoint, DriveSubsystem driveSubsystem, LedPosition ledPosition) {
        this(waypoint, driveSubsystem);
        mLedPosition = ledPosition;
    }

    public SetWaypointCommand(Pose2d waypoint, DriveSubsystem driveSubsystem) {
        mDrive = driveSubsystem;
        mWaypoint = waypoint;

        mLedPosition = null;
    }

    @Override
    public void execute() {
        mDrive.setWaypoint(mWaypoint);
        mDrive.setLedPosition(mLedPosition);

        if (mLedPosition != null) {
            ButtonBoxPublisher.blinkLed(mLedPosition);
        }
    }

}
