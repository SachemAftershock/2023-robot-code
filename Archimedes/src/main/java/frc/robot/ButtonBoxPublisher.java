package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.ErrorTracker.ErrorType;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import frc.robot.enums.ButtonBoxLedInfo.LedState;

public class ButtonBoxPublisher {

    private static final StringPublisher sWaypointPub = NetworkTableInstance.getDefault()
        .getStringTopic("/ButtonBox/Waypoint").publish();
    private static final StringPublisher sIntakePub = NetworkTableInstance.getDefault()
        .getStringTopic("/ButtonBox/Intake").publish();
    private static final StringPublisher sSuperstructurePub = NetworkTableInstance.getDefault()
        .getStringTopic("/ButtonBox/Superstructure").publish();
    private static final StringPublisher sTogglePub = NetworkTableInstance.getDefault()
        .getStringTopic("/ButtonBox/Toggle").publish();
    private static final StringPublisher sJoystickEnablePub = NetworkTableInstance.getDefault()
        .getStringTopic("/ButtonBox/Joystick").publish();
    private static final StringPublisher sMessagePub = NetworkTableInstance.getDefault()
        .getStringTopic("/ButtonBox/Message").publish();
    private static final StringPublisher sAllLedPub = NetworkTableInstance.getDefault()
        .getStringTopic("/ButtonBox/AllLed").publish();

    private ButtonBoxPublisher() {
    }

    public static void enableLed(LedPosition ledPosition) {
        setLed(ledPosition, LedState.eOn);
    }

    public static void disableLed(LedPosition ledPosition) {
        setLed(ledPosition, LedState.eOff);
    }

    public static void blinkLed(LedPosition ledPosition) {
        setLed(ledPosition, LedState.eBlink);
    }

    public static void enableAllLeds() {
        setAllLeds(LedState.eOn);
    }

    public static void disableAllLeds() {
        setAllLeds(LedState.eOff);
    }

    public static void blinkAllLeds() {
        setAllLeds(LedState.eBlink);
    }

    public static void sendMessage(String message) {

        if (message == null) {
            System.out.println("null message");
            return;
        }

        sMessagePub.set(String.format("msg %s\0", message));
    }

    public static void sendMessage(String message, int line) {
        if (line > 2 || line < 0) {
            System.out.println("invalid line number");
            return;
        }

        if (message == null) {
            System.out.println("null message");
            return;
        }

        sMessagePub.set(String.format("msg %s %d\0", message, line));
    }

    public static void sendError(ErrorType errorType) {
        if (errorType == null) {
            System.out.println("null error type");
            return;
        }

        switch (errorType) {
            case eArmLidarInfinity:
                sendMessage("Arm Lidar Inf", 2);
                break;
            case eElevatorLidarInfinity:
                sendMessage("Elev Lidar Inf", 2);
                break;
            case eIntakeLidarInfinity:
                sendMessage("Intake Lidar Inf", 2);
                break;
            case eElevatorMoveFailure:
                sendMessage("Elev Move Fail", 2);
                break;
            case eNavXZero:
                sendMessage("NavX Zero", 2);
                break;
            default:
                break;
        }

    }

    private static void setLed(LedPosition ledPosition, LedState ledState) {
        if (ledPosition == null) {
            sendMessage("null led position");
            System.out.println("null led position");
            return;
        }

        getPublisher(ledPosition).set(String.format("led set %d %s\0", ledPosition.getId(), ledState.getState()));
    }

    private static void setAllLeds(LedState ledState) {
        sAllLedPub.set(String.format("led set all %s\0", ledState.getState()));
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
