package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.enums.ButtonBoxLedInfo.LedState;

public class ButtonBoxPublisher {

    private static final StringPublisher sPub = NetworkTableInstance.getDefault().getStringTopic("/ButtonBox/LedCommand").publish();

    private ButtonBoxPublisher() {
    }

    public static void setLed(LedPosition ledPosition, LedState ledState) {
        sPub.set(String.format("led set %d %d", ledPosition.getId(), ledState.getState()));
    }

    public static void setAllLeds(LedState ledState) {
        sPub.set(String.format("led setall %d", ledState.getState()));
    }

    public static void sendMessage(String message) {
        sPub.set(String.format("msg %s", message));
    }

}
