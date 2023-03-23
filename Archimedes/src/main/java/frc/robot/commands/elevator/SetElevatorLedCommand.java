package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.ButtonBoxPublisher;
import frc.robot.enums.SuperState;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;

public class SetElevatorLedCommand extends InstantCommand {

    private SuperState mSuperState;

    public SetElevatorLedCommand(SuperState superState) {
        mSuperState = superState;
    }

    @Override
    public void execute() {
        LedPosition position;

        switch (mSuperState) {
            case eStow:
                position = LedPosition.eStow;
                break;
            case eLow:
                position = LedPosition.eLow;
                break;
            case eMid:
                position = LedPosition.eMid;
                break;
            case eHigh:
                position = LedPosition.eHigh;
                break;
            case ePlayerStation:
                position = LedPosition.ePlayerStation;
                break;
            default:
                position = null;
                break;
        }

        ButtonBoxPublisher.enableLed(position);
    }

}
