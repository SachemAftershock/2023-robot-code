package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandJoystick extends CommandGenericHID {

    public CommandJoystick(int port) {
        super(port);
    }

    public double getX() {
        return this.getRawAxis(AxisType.kX.value);
    }
    
    public double getY() {
        return getRawAxis(AxisType.kY.value);
    }

    public double getTwist() {
        return getRawAxis(AxisType.kZ.value);
    }

    public boolean getTriggerPressed() {
        return this.getHID().getRawButton(Joystick.ButtonType.kTrigger.value);
    }

    public Trigger getTrigger() {
        return this.button(Joystick.ButtonType.kTrigger.value);
    }
}
