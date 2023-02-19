package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static class DriveConstants {

    public static enum CardinalDirection {
      eX, eY
    }

    public static final double kDriveControllerDeadband = 0.05;
    public static final boolean kSquareAxis = true;

    public static final double[] kDriveAngularGains = { 0.02, 0.0, 0.0 }; // dont use I it sucks - Shreyas
    public static final double[] kDriveLinearGains = { 0.4, 0.0, 0.0 };

    public static final double kPX = 1.25;
    public static final double kPY = 1.25;

    public static final double kAutoRotateEpsilon = 3.0;
    public static final double kLinearDriveEpsilon = 0.0;

    public static final double kDrivetrainTrackwidthMeters = 0.5461;
    public static final double kDrivetrainWheelbaseMeters = 0.5461;

    // angles in radians.
    // to convert from degrees to radians multiply by pi/180
    public static final double kFrontLeftSteerOffset = -0.35 - (Math.PI / 2.0);// -.35;
    public static final double kFrontRightSteerOffset = 0.4 - (Math.PI / 2.0);// 0.40;
    public static final double kBackLeftSteerOffset = 0.45 - (Math.PI / 2.0);// .45;
    public static final double kBackRightSteerOffset = -0.5 - (Math.PI / 2.0);// -.5;

    private static final double kMk4L1DriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
    private static final double kMk4WheelDiameter = 0.10033;

    public static final double kMaxVelocityMetersPerSecond = 6380.0 / 60.0 * kMk4L1DriveReduction * kMk4WheelDiameter * Math.PI;

    // TODO: Change
    public static final double kMaxAccelerationMetersPerSecondSquared = kMaxVelocityMetersPerSecond * 0.25;

    public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond
      / Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0);

    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
  }

  public static class VisionConstants {
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    // Cam mounted facing forward, half
    // a meter forward of center, half
    // a meter up
    // from center

    public static final String cameraName = "CameraName";
  }

  public static class ControllerConstants {
    public static final int kPrimaryThrottleControllerPort = 0;
    public static final int kPrimaryTwistControllerPort = 1;
    public static final int kButtonBoxPort = 2;

  }

  public static class IntakeConstants {
    public static final double kIngestConeSpeed = 0.5;
    public static final double kIngestCubeSpeed = 0.5;
  }

  public static class ArmConstants {
    public static final double kMaxVelocityRadPerSecond = 0;
    public static final double kArmOffsetRads = 0;
    public static final double kMaxAccelerationRadPerSecSquared = 0;
    public static final double[] kGains = { 0.0, 0.0, 0.0 };
    public static final double kIntegralZone = 0.0;
    public static final double kDt = 0.02;
  }

  public static class ElevatorConstants {
    public static final double[] kPidGains = { 0.0, 0.0, 0.0 };
    public static final double kEpsilon = 0.0;
  }
}
