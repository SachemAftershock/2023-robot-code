package frc.robot.enums;

public class ButtonBoxLedInfo {

    public enum LedState {
        eOff("off"), eOn("on"), eBlink("blink");

        private String mState;

        LedState(String state) {
            mState = state;
        }

        public String getState() {
            return mState;
        }
    }

    public enum LedPosition {
        // WAYPOINTS
        eDriveTo1(5), eDriveTo2(23), eDriveTo3(7), eDriveTo4(4), eDriveTo5(17), eDriveTo6(15), eDriveTo7(6), eDriveTo8(18), eDriveTo9(3), eHumanPlayerLeft(14),
        eHumanPlayerRight(25), eCancel(16),

        // INTAKE
        eIngest(13), eEject(19),

        // SUPERSTRUCTURE
        eStow(11), eLow(22), eMid(24), eHigh(12), ePlayerStation(10),

        // TOGGLE
        eCubeActive(2), eConeActive(9),

        // JOYSTICK
        eJoystickEnable(8);

        private int mLedId;

        private LedPosition(int ledId) {
            mLedId = ledId;
        }

        public int getId() {
            return mLedId;
        }
    }

    public enum ButtonPosition {
        // WAYPOINTS
        eDriveTo1, eDriveTo2, eDriveTo3, eDriveTo4, eDriveTo5, eDriveTo6, eDriveTo7, eDriveTo8, eDriveTo9, eCancel, eHumanPlayerLeft, eHumanPlayerRight,

        // Toggle
        eCubeActive, eConeActive,

        // INTAKE
        eIngest, eEject,

        // SUPERSTRUCTURE
        eStow, ePlayerStation, eLow, eMid, eHigh,

        // JOYSTICK
        eJoystickEnable;

        public int getId() {
            return this.ordinal() + 1;
        }

        public static ButtonPosition getButtonPosition(int id) {
            return ButtonPosition.values()[id - 1];
        }
    }

}
