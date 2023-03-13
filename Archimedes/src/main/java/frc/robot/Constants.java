package frc.robot;

import java.util.List;
import java.util.TreeMap;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.enums.ButtonBoxLedInfo.ButtonPosition;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;

import java.util.HashMap;

public class Constants {

    public static class DriveConstants {

        public static enum CardinalDirection {
            eX, eY
        }

        public static final double kDriveRateLimit = 0.25;

        public static final double kDriveControllerDeadband = 0.05;
        public static final boolean kSquareAxis = true;

        public static final double kDriveSpeedScaleFactor = 0.5;

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
        public static final double kFrontLeftSteerOffset = -0.35 + (Math.PI / 2.0); // - Math.toRadians(15);// - (Math.PI / 2.0);// -.35;
        public static final double kFrontRightSteerOffset = 0.4 + (Math.PI / 2.0); // - Math.toRadians(15);; //- (Math.PI / 2.0);// 0.40;
        public static final double kBackLeftSteerOffset = 0.45 + (Math.PI / 2.0); // - Math.toRadians(15);; //- (Math.PI / 2.0);// .45;
        public static final double kBackRightSteerOffset = -0.5 + (Math.PI / 2.0); // - Math.toRadians(15);; //- (Math.PI / 2.0);// -.5;

        private static final double kMk4L1DriveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
        private static final double kMk4WheelDiameter = 0.10033;

        public static final double kMaxVelocityMetersPerSecond = 6380.0 / 60.0 * kMk4L1DriveReduction * kMk4WheelDiameter * Math.PI;

        // TODO: Change
        public static final double kMaxAccelerationMetersPerSecondSquared = kMaxVelocityMetersPerSecond * 0.25;

        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond
            / Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0);

        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

        public static double kRotationScalingConstant = 0.3;

        // TODO: Consider if we need this
        public static double kMinimumDistanceForAutoDrive = 0.0;
        public static double kDriveToTargetEpsilon = 0.1;

        public static final double kMinBalanceAngle = 9.0;//In degrees
        public static final double kMaxBalanceAngle = 16.0; 

        public static final double[] kBalanceRobotGains = {0.0, 0.0, 0.0};
        public static final double kBalanceRobotEpsilon = 1.0;

        public static final double kDriveSpeed = 0.5; // adjust as needed
        public static final double kBalanceThreshold = 2.5; // degrees
        
        public static final double kChargeStationWidth = 247; // cm
        public static final double kChargeStationDepth = 193; // cm
        public static final double kChargeStationLevelHeight = 23; // cm
        public static final double kChargeStationMaxTilt = 15; // degrees
        
        public static final double kRampLength = 39; // cm
        public static final double kRampLevelAngle = 34.25; // degrees
        public static final double kRampMaxTiltAngle = 71.5; // degrees
        public static final double kTargetBalanceAngle = 0;
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

    public static final class ButtonBoxConstants {
        public static final HashMap<ButtonPosition, LedPosition> kButtonBoxButtonMap = new HashMap<ButtonPosition, LedPosition>();

        static {
            kButtonBoxButtonMap.put(ButtonPosition.eDriveTo1, LedPosition.eDriveTo1); // drive to 1
            kButtonBoxButtonMap.put(ButtonPosition.eDriveTo2, LedPosition.eDriveTo2); // drive to 2
            kButtonBoxButtonMap.put(ButtonPosition.eDriveTo3, LedPosition.eDriveTo3); // drive to 3
            kButtonBoxButtonMap.put(ButtonPosition.eDriveTo4, LedPosition.eDriveTo4); // drive to 4
            kButtonBoxButtonMap.put(ButtonPosition.eDriveTo5, LedPosition.eDriveTo5); // drive to 5
            kButtonBoxButtonMap.put(ButtonPosition.eDriveTo6, LedPosition.eDriveTo6); // drive to 6
            kButtonBoxButtonMap.put(ButtonPosition.eDriveTo7, LedPosition.eDriveTo7); // drive to 7
            kButtonBoxButtonMap.put(ButtonPosition.eDriveTo8, LedPosition.eDriveTo8); // drive to 8
            kButtonBoxButtonMap.put(ButtonPosition.eDriveTo9, LedPosition.eDriveTo9); // drive to 9
            kButtonBoxButtonMap.put(ButtonPosition.eCancel, LedPosition.eCancel); // cancel
            kButtonBoxButtonMap.put(ButtonPosition.eHumanPlayerLeft, LedPosition.eHumanPlayerLeft); // human player left
            kButtonBoxButtonMap.put(ButtonPosition.eHumanPlayerRight, LedPosition.eHumanPlayerRight); // human player right
            kButtonBoxButtonMap.put(ButtonPosition.eCubeActive, LedPosition.eCubeActive); // toggle cube
            kButtonBoxButtonMap.put(ButtonPosition.eConeActive, LedPosition.eConeActive); // toggle cone
            kButtonBoxButtonMap.put(ButtonPosition.eIngest, LedPosition.eIngest); // ingest
            kButtonBoxButtonMap.put(ButtonPosition.eEject, LedPosition.eEject); // eject
            kButtonBoxButtonMap.put(ButtonPosition.eStow, LedPosition.eStow); // stow
            kButtonBoxButtonMap.put(ButtonPosition.ePlayerStation, LedPosition.ePlayerStation); // human player
            kButtonBoxButtonMap.put(ButtonPosition.eLow, LedPosition.eLow); // low
            kButtonBoxButtonMap.put(ButtonPosition.eMid, LedPosition.eMid); // mid
            kButtonBoxButtonMap.put(ButtonPosition.eHigh, LedPosition.eHigh); // high
            kButtonBoxButtonMap.put(ButtonPosition.eJoystickEnable, LedPosition.eJoystickEnable); // enable joystick
        }
    }

    public static class IntakeConstants {
        public static final double kIngestConeSpeed = 0.5;
        public static final double kIngestCubeSpeed = 0.5;
        public static final double kIntakeWidth = 12.0;
        // public static final int kIntakeLidarId = 0;
    }

    public static class ArmConstants {

        public static final double kCubeOffset = 0.0;
        public static final double kMaxVelocityMeterPerSecond = 0.05;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.025;
        public static final double kArmOffsetRads = 0;
        public static final double[] kGains = { 2.0, 0.0, 0.0 };// { 1.2, 0.0, 0.0 };
        public static final double kIntegralZone = 0.0;
        public static final double kDt = 0.02;

        public static final double kEpsilon = 0.15;

        public static final double kJogSpeed = 0.2;
        public static final double kArmLidarOffset = -3.0;

        public static final double[][] kBarDistanceToArmExtension = { { 18, 17.25 }, { 17, 29 }, { 16, 34.75 }, { 15, 42 }, { 13, 49 }
                // {13, 48.5},
        };

        public static final int kBarDistanceIndex = 0;
        public static final int kArmDistanceIndex = 1;

        public static double getBarDistance(double desiredArmExtension) {
            if (desiredArmExtension < kBarDistanceToArmExtension[0][kArmDistanceIndex]) {
                return -1;
            }

            for (int i = 1; i < kBarDistanceToArmExtension.length; ++i) {
                if (desiredArmExtension < kBarDistanceToArmExtension[i][kArmDistanceIndex]) {
                    // x and y are flipped of what you expect bc this is a "reverse" look up
                    // We know the desired arm distance, and have to map it to a interpolated bar distance
                    
                    double x1 = kBarDistanceToArmExtension[i][kArmDistanceIndex];
                    double x0 = kBarDistanceToArmExtension[i - 1][kArmDistanceIndex];

                    double y1 = kBarDistanceToArmExtension[i][kBarDistanceIndex];
                    double y0 = kBarDistanceToArmExtension[i - 1][kBarDistanceIndex];
                    double a = y1 - y0;
                    double b = x1 - x0;
                    double slope = a/b;
                    double c = y1 - (slope*x1);
                    // System.out.println(slope + "    -----  " + slope * desiredArmExtension + y0);

                    return (desiredArmExtension - c) / slope;//slope * desiredArmExtension + y0; 
                }
            }

            return -1;
        }
    }

    public static class ElevatorConstants {
        public static final double[] kPidGains = { 0.06, 0.0, 0.0 };
        public static final double[] kTrapezoidalPidGains = { 0.05, 0.0, 0.0 };
        public static final double kMaxVelocityMeterPerSecond = 0.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
        public static final double kEpsilon = 1.5;

        public static final double kJogSpeed = 0.2;

        public static final double kElevatorLidarOffset = 0.0;
        public static final double kElevatorLidarHeightFromGround = 3.75;
        public static final int kElevatorMedianFilterSampleSize = 10;
        public static final double kElevatorMaxHeight = 64.0; 
        public static final double kElevatorMinHeight = 17.0; 
        public static final double kCompensatingVoltage = 0.0;

    }

    /*
     * Team 2539 Constants
     * https://github.com/FRC2539/javabot-2023/blob/main/src/main/java/frc/robot
     */
    public static final class FieldConstants {
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);

        public static final double robotLengthWithBumpers = Units.inchesToMeters(30 + 8);

        /* X Placement constants from 6328 */
        public static final double outerX = Units.inchesToMeters(54.25);
        public static final double lowX = outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
        public static final double midX = outerX - Units.inchesToMeters(22.75);
        public static final double highX = outerX - Units.inchesToMeters(39.75);

        /* Z Placement constants from 6328 */
        public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
        public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
        public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
        public static final double highConeZ = Units.inchesToMeters(46.0);
        public static final double midConeZ = Units.inchesToMeters(34.0);

        public static class PlacementLocation {
            public Pose2d robotPlacementPose;
            public boolean isCone;

            public PlacementLocation(Pose2d poseAlignedWithEdge, double lengthOfRobotWithBumpers, boolean isCone) {
                var transformHybridToRobot = new Transform2d(new Translation2d(lengthOfRobotWithBumpers / 2, 0), Rotation2d.fromDegrees(180));
                robotPlacementPose = poseAlignedWithEdge.transformBy(transformHybridToRobot);
                this.isCone = isCone;
            }

            public Pose3d getHybridPose() {
                return new Pose3d(lowX, robotPlacementPose.getY(), 0, new Rotation3d());
            }

            public Pose3d getMidPose() {
                return new Pose3d(midX, robotPlacementPose.getY(), isCone ? midConeZ : midCubeZ, new Rotation3d());
            }

            public Pose3d getHighPose() {
                return new Pose3d(highX, robotPlacementPose.getY(), isCone ? highConeZ : highCubeZ, new Rotation3d());
            }
        }

        public static final int numberOfNodeRows = 9;
        public static final double separationBetweenNodeRows = Units.inchesToMeters(22.0);
        public static final Pose2d firstPlacingPose = new Pose2d(outerX, Units.inchesToMeters(20.19), new Rotation2d());

        public static final boolean[] isCone = new boolean[] { true, false, true, true, false, true, true, false, true };

        // Store the locations we will score from on the field (for automatic placement)
        public static final PlacementLocation[] kPlacingPoses = new PlacementLocation[numberOfNodeRows];

        static {
            for (int i = 0; i < numberOfNodeRows; i++) {
                kPlacingPoses[i] = new PlacementLocation(
                    new Pose2d(firstPlacingPose.getX(), firstPlacingPose.getY() + i * separationBetweenNodeRows, new Rotation2d()), robotLengthWithBumpers,
                    isCone[i]
                );
            }
        }

        private static TreeMap<Double, PlacementLocation> locationMap = new TreeMap<>();

        static {
            for (PlacementLocation placementLocation : kPlacingPoses) {
                locationMap.put(placementLocation.robotPlacementPose.getY(), placementLocation);
            }
        }

        /**
         * Finds the game piece placement area closest to the robot.
         * 
         * @param robotPose
         * @return The nearest placement location
         */
        public static PlacementLocation getNearestPlacementLocation(Pose2d robotPose) {
            double target = robotPose.getY();
            Double lowerYValue = locationMap.floorKey(target);
            Double upperYValue = locationMap.ceilingKey(target);

            // Account for the pose being below or above the range
            if (lowerYValue == null) return locationMap.get(upperYValue);
            else if (upperYValue == null) return locationMap.get(lowerYValue);

            boolean isLowerCloser = Math.abs(target - lowerYValue) < Math.abs(target - upperYValue);

            return isLowerCloser ? locationMap.get(lowerYValue) : locationMap.get(upperYValue);
        }

        public static final List<AprilTag> kAprilTags = List.of(
            new AprilTag(
                1, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))
            ),
            new AprilTag(
                2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))
            ),
            new AprilTag(
                3, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))
            ),
            new AprilTag(
                4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, Math.PI))
            ), new AprilTag(5, new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d())),
            new AprilTag(6, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d())),
            new AprilTag(7, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d())),
            new AprilTag(8, new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22), new Rotation3d()))
        );

        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = new AprilTagFieldLayout(kAprilTags, fieldLength, fieldWidth);

        public static void setAprilTagOrigin() {
            APRIL_TAG_FIELD_LAYOUT
                .setOrigin(DriverStation.getAlliance() == Alliance.Red ? OriginPosition.kRedAllianceWallRightSide : OriginPosition.kBlueAllianceWallRightSide);
        }
    }

    public static final class LoadingZone {
        // Region dimensions
        public static final double width = Units.inchesToMeters(99.0);
        public static final double innerX = FieldConstants.fieldLength;
        public static final double midX = FieldConstants.fieldLength - Units.inchesToMeters(132.25);
        public static final double outerX = FieldConstants.fieldLength - Units.inchesToMeters(264.25);
        public static final double leftY = FieldConstants.fieldWidth;
        public static final double midY = leftY - Units.inchesToMeters(50.5);
        public static final double rightY = leftY - width;
        public static final Translation2d[] regionCorners = new Translation2d[] { new Translation2d(midX, rightY), // Start at lower left
                                                                                                                   // next to border with
                                                                                                                   // opponent community
                new Translation2d(midX, midY), new Translation2d(outerX, midY), new Translation2d(outerX, leftY), new Translation2d(innerX, leftY),
                new Translation2d(innerX, rightY), };

        // Double substation dimensions
        public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
        public static final double doubleSubstationX = innerX - doubleSubstationLength;
        public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);
        public static final double doubleSubstationCenterY = Units.inchesToMeters(265.74);
        public static final Pose2d kDoubleSubstationPose = new Pose2d(doubleSubstationX, doubleSubstationCenterY, Rotation2d.fromDegrees(0));

        // Single substation dimensions
        public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
        public static final double singleSubstationLeftX = FieldConstants.fieldLength - doubleSubstationLength - Units.inchesToMeters(88.77);
        public static final double singleSubstationCenterX = singleSubstationLeftX + (singleSubstationWidth / 2.0);
        public static final double singleSubstationRightX = singleSubstationLeftX + singleSubstationWidth;
        public static final Translation2d singleSubstationTranslation = new Translation2d(singleSubstationCenterX, leftY);

        public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
        public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
        public static final double singleSubstationCenterZ = singleSubstationLowZ + (singleSubstationHeight / 2.0);
        public static final double singleSubstationHighZ = singleSubstationLowZ + singleSubstationHeight;
    }

    // Locations of staged game pieces
    public static final class StagingLocations {
        public static final double centerOffsetX = Units.inchesToMeters(47.36);
        public static final double positionX = FieldConstants.fieldLength / 2.0 - Units.inchesToMeters(47.36);
        public static final double firstY = Units.inchesToMeters(36.19);
        public static final double separationY = Units.inchesToMeters(48.0);
        public static final Translation2d[] translations = new Translation2d[4];

        static {
            for (int i = 0; i < translations.length; i++) {
                translations[i] = new Translation2d(positionX, firstY + (i * separationY));
            }
        }
    }
}