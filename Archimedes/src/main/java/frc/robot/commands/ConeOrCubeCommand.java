package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ConeOrCubeCommand extends InstantCommand{
private static boolean mIsCone = false;

public ConeOrCubeCommand(boolean isCone)
{
    mIsCone = isCone;

}

@Override
public void initialize() {
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  mIsCone = !mIsCone;
  System.out.println("execute Cone: " + mIsCone);
}

@Override
public void end(boolean interrupted) {

}
/**
 * 
 * @return returns if its a cone or not a cone
 */
public static boolean getisCone()
{
    return mIsCone;
}

}
