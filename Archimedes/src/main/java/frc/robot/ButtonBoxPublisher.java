package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.enums.ButtonBoxLedInfo.LedState;

public class ButtonBoxPublisher {

    private static final StringPublisher sWaypointPub = NetworkTableInstance.getDefault().getStringTopic("/ButtonBox/Waypoint").publish();
    private static final StringPublisher sIntakePub = NetworkTableInstance.getDefault().getStringTopic("/ButtonBox/Intake").publish();
    private static final StringPublisher sSuperstructurePub = NetworkTableInstance.getDefault().getStringTopic("/ButtonBox/Superstructure").publish();
    private static final StringPublisher sTogglePub = NetworkTableInstance.getDefault().getStringTopic("/ButtonBox/Toggle").publish();
    private static final StringPublisher sJoystickEnablePub = NetworkTableInstance.getDefault().getStringTopic("/ButtonBox/Joystick").publish();
    private static final StringPublisher sMessagePub = NetworkTableInstance.getDefault().getStringTopic("/ButtonBox/Message").publish();

    private ButtonBoxPublisher() {}

    public static void enableLed(LedPosition ledPosition) {
        setLed(ledPosition, LedState.eOn);
    }

    public static void disableLed(LedPosition ledPosition) {
        setLed(ledPosition, LedState.eOff);
    }

    public static void blinkLed(LedPosition ledPosition) {
        setLed(ledPosition, LedState.eBlink);
    }

    private static void setLed(LedPosition ledPosition, LedState ledState) {
        if (ledPosition == null) return;
        
        getPublisher(ledPosition).set(String.format("led set %d %s\0", ledPosition.getId(), ledState.getState()));
    }

    public static void sendMessage(String message) {
        sMessagePub.set(String.format("msg %s\0", message));
    }

    private static StringPublisher getPublisher(LedPosition ledPosition) {
        int ord = ledPosition.ordinal();
        if (ord < 12) return sWaypointPub;
        if (ord < 14) return sIntakePub;
        if (ord < 19) return sSuperstructurePub;
        if (ord < 21) return sTogglePub;
        return sJoystickEnablePub;
    }
}
