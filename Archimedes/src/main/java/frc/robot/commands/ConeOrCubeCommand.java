package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ConeOrCubeCommand{
private static boolean isCone;
public ConeOrCubeCommand(boolean isCone)
{
    this.isCone = isCone;
}
public boolean getisCone()
{
    return isCone;
}

}
