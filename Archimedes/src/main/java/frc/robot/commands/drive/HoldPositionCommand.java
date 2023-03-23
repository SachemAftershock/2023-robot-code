package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class HoldPositionCommand extends CommandBase {

    private DriveSubsystem mDrive;

    public HoldPositionCommand(DriveSubsystem drive) {
        mDrive = drive;
        addRequirements(mDrive);
    }

    @Override
    public void initialize() {
        mDrive.lockWheels();
    }

    @Override
    public void end(boolean interrupted) {
        mDrive.unlockWheels();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}