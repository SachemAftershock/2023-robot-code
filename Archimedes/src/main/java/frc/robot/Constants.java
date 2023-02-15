package frc.robot;

public class Constants {


    public static class ArmConstants {

        public static final int kArmMotorID = 5;
        public static final double kP = 0.5; 
        public static final double kI = 1e-6;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kInchesToRotations = 1*4;
        public static final double kDt = 0.02;
        public static final double kMaxVelocity = 1.75;
        public static final double kMaxAcceleration = 0.75;

    }

    public static enum ButtonState {
        eCone, eCube, eNone, eHigh, eMid, eFloor, eHumanStation, ePark, eIntake, eEject
    }

    public static enum DriveLocationLUT {

        eSlot1(7.2, -2.9), eSlot2(0.0, 0.0), eSlot3(0.0, 0.0),
        eSlot4(0.0, 0.0), eSlot5(0.0, 0.0), eSlot6(0.0, 0.0),
        eSlot7(0.0, 0.0), eSlot8(0.0, 0.0), eSlot9(0.0, 0.0), 
        
        eHumanStation(0.0, 0.0), eNone(0.0, 0.0);

        private double mXCoord;
        private double mYCoord;

        private DriveLocationLUT(double x, double y) {
            mXCoord = x;
            mYCoord = y;
        }

        public double getXCoord() {
            return mXCoord;
        }

        public double getYCoord() {
            return mYCoord;
        }

    }

    public static enum ElevatorStateLUT{

        eFloorCube(0.0,0.0),  
        eFloorCone(0.0,0.0),

        eHumanStationCone(0.0,0.0), 
        eHumanStationCube(0.0,0.0),

        eMidCone(0.0, 0.0),
        eMidCube(0.0, 0.0),

        eHighCone(0.0, 0.0),
        eHighCube(0.0, 0.0),
        
        ePark(0.0,0.0);

        private double mElevatorHeight;
        private double mArmExtension;

        private ElevatorStateLUT(double elevatorHeight, double armExtension) {
            mElevatorHeight = elevatorHeight;
            mArmExtension = armExtension;
        }

        public double getElevatorHeight() {
            return mElevatorHeight;
        }

        public double getArmExtension() {
            return mArmExtension;
        }

    }
    
}
