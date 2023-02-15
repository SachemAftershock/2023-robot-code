package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ConeOrCubeCommand extends InstantCommand{
private static boolean isCone;

public ConeOrCubeCommand(boolean isCone)
{
    this.isCone = isCone;
}
public static boolean getisCone()
{
    return isCone;
}

}
