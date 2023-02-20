package frc.robot.enums;

public class ButtonBoxLedInfo {

    public enum LedState {
        eOff, eOn, eBlink;

        public int getState() {
            return ordinal();
        }
    }

    public enum LedPosition {
        eDriveTo1(0), eDriveTo2(1);

        private int mLedId;

        private LedPosition(int ledId) {
            mLedId = ledId;
        }

        public int getId() {
            return mLedId;
        }
    }

}
