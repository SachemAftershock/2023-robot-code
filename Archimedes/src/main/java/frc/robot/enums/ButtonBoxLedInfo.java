package frc.robot.enums;

public class ButtonBoxLedInfo {

    public enum LedState {
        eOff, eOn, eBlink;

        public int getState() {
            return ordinal();
        }
    }

    public enum LedPosition {
        // WAYPOINTS
        eDriveTo1(0), eDriveTo2(1), eDriveTo3(2),
        eDriveTo4(3), eDriveTo5(4), eDriveTo6(5),
        eDriveTo7(6), eDriveTo8(7), eDriveTo9(8),
        eHumanPlayerLeft(9), eHumanPlayerRight(10), eCancel(11),

        // INTAKE
        eIngest(12), eEject(13),
        
        // SUPERSTRUCTURE
        eStow(14), eLow(15), eMid(16), eHigh(17), ePlayerStation(18),
        
        // TOGGLE
        eCubeActive(19), eConeActive(20),
        
        // JOYSTICK
        eJoystickEnable(21);

        private int mLedId;

        private LedPosition(int ledId) {
            mLedId = ledId;
        }

        public int getId() {
            return mLedId;
        }
    }

}
