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
    public static class DriveConstants {

        public static enum CardinalDirection {
            eX, eY
        }

        public static final double kDriveControllerDeadband = 0.05;
        public static final boolean kSquareAxis = true; 

        public static final double[] kDriveAngularGains = {0.02, 0.0, 0.0}; //dont use I it sucks - Shreyas
        public static final double[] kDriveLinearGains = {0.4, 0.0, 0.0};

        public static final double kPX = 1.25;
        public static final double kPY = 1.25;

        public static final double kAutoRotateEpsilon = 3.0;
        public static final double kLinearDriveEpsilon = 0.0;
        
        
        public static final double kDrivetrainTrackwidthMeters = 0.5461;
        public static final double kDrivetrainWheelbaseMeters = 0.5461;
        
        // angles in radians. 
        // to convert from degrees to radians multiply by pi/180 
        public static final double kFrontLeftSteerOffset = -0.35 - (Math.PI / 2.0);//-.35;
        public static final double kFrontRightSteerOffset = 0.4 - (Math.PI / 2.0);//0.40;
        public static final double kBackLeftSteerOffset = 0.45 - (Math.PI / 2.0);//.45;
        public static final double kBackRightSteerOffset = -0.5 - (Math.PI / 2.0);//-.5;


        private static final double kMk4L1DriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
        private static final double kMk4WheelDiameter = 0.10033;

        
        public static final double kMaxVelocityMetersPerSecond = 6380.0 / 60.0 *
            kMk4L1DriveReduction * kMk4WheelDiameter * Math.PI;

        //TODO: Change
        public static final double kMaxAccelerationMetersPerSecondSquared = kMaxVelocityMetersPerSecond * 0.25;

        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond /
        Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0);

        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
    }

    
}
