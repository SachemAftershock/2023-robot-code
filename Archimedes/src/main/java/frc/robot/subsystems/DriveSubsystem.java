package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.lib.AftershockSubsystem;
import frc.lib.Limelight;
import frc.lib.Limelight.FluidicalPoseInfo;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

//import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Ports.DrivePorts.*;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

public class DriveSubsystem extends AftershockSubsystem {
	private static DriveSubsystem mInstance;

	/**
	 * The maximum voltage that will be delivered to the drive motors.
	 * <p>
	 * This can be reduced to cap the robot's maximum speed. Typically, this is
	 * useful during initial testing of the robot.
	 */
	public static final double MAX_VOLTAGE = 12.0;
	// FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
	// The formula for calculating the theoretical maximum velocity is:
	// <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
	// pi
	// By default this value is setup for a Mk3 standard module using Falcon500s to
	// drive.
	// An example of this constant for a Mk4 L2 module with NEOs to drive is:
	// 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
	// SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
	/**
	 * The maximum velocity of the robot in meters per second.
	 * <p>
	 * This is a measure of how fast the robot should be able to drive in a straight
	 * line.
	 */
	// public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
	// SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
	// SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
	/**
	 * The maximum angular velocity of the robot in radians per second.
	 * <p>
	 * This is a measure of how fast the robot can rotate in place.
	 */
	// Here we calculate the theoretical maximum angular velocity. You can also
	// replace this with a measured amount.
	// public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
	// MAX_VELOCITY_METERS_PER_SECOND /
	// Math.hypot(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters /
	// 2.0);
	
	private final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(
		// Front left
		new Translation2d(kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0),
		// Front right
		new Translation2d(kDrivetrainTrackwidthMeters / 2.0, -kDrivetrainWheelbaseMeters / 2.0),
		// Back left
		new Translation2d(-kDrivetrainTrackwidthMeters / 2.0, kDrivetrainWheelbaseMeters / 2.0),
		// Back right
		new Translation2d(-kDrivetrainTrackwidthMeters / 2.0, -kDrivetrainWheelbaseMeters / 2.0)
	);

	private final AHRS mNavx; // NavX connected over MXP

	private final SwerveDrivePoseEstimator mPoseEstimator;
	// private final PhotonCamera mPhotonCamera;

	private final SwerveModule mFrontLeftModule;
	private final SwerveModule mFrontRightModule;
	private final SwerveModule mBackLeftModule;
	private final SwerveModule mBackRightModule;

	private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	private final Limelight mLimelight;

	private Pose2d mWaypoint;
	private LedPosition mLedPosition;

	private DriveSubsystem() {
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		mNavx = new AHRS(SPI.Port.kMXP, (byte) 200);

		// mPhotonCamera = new PhotonCamera("photonvision");

		mFrontLeftModule = Mk4SwerveModuleHelper.createFalcon500Neo(
			// This parameter is optional, but will allow you to see the current state of
			// the module on the dashboard.
			tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
			// This can either be STANDARD or FAST depending on your gear configuration
			Mk4SwerveModuleHelper.GearRatio.L1,
			// This is the ID of the drive motor
			kFrontLeftDriveMotorId,
			// This is the ID of the steer motor
			kFrontLeftSteerMotorId,
			// This is the ID of the steer encoder
			kFrontLeftSteerEncoderId,
			// This is how much the steer encoder is offset from true zero (In our case,
			// zero is facing straight forward)
			kFrontLeftSteerOffset
		);

		mFrontRightModule = Mk4SwerveModuleHelper.createFalcon500Neo(
			tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0), Mk4SwerveModuleHelper.GearRatio.L1,
			kFrontRightDriveMotorId, kFrontRightSteerMotorId, kFrontRightSteerEncoderId, kFrontRightSteerOffset
		);

		mBackLeftModule = Mk4SwerveModuleHelper.createFalcon500Neo(
			tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0), Mk4SwerveModuleHelper.GearRatio.L1,
			kBackLeftDriveMotorId, kBackLeftSteerMotorId, kBackLeftSteerEncoderId, kBackLeftSteerOffset
		);

		mBackRightModule = Mk4SwerveModuleHelper.createFalcon500Neo(
			tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0), Mk4SwerveModuleHelper.GearRatio.L1,
			kBackRightDriveMotorId, kBackRightSteerMotorId, kBackRightSteerEncoderId, kBackRightSteerOffset
		);

		mFrontLeftModule.setCanStatusFramePeriodReductions();
		mFrontRightModule.setCanStatusFramePeriodReductions();
		mBackLeftModule.setCanStatusFramePeriodReductions();
		mBackRightModule.setCanStatusFramePeriodReductions();

		mPoseEstimator = new SwerveDrivePoseEstimator(mKinematics, new Rotation2d(), getPositions(), new Pose2d());

		mLimelight = new Limelight("limelight");

	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the
	 * robot is currently facing to the 'forwards' direction.
	 */
	public void zeroGyroscope() {
		mNavx.zeroYaw();
	}

	public Rotation2d getGyroscopeRotation() {
		if (mNavx.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(mNavx.getFusedHeading());
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - mNavx.getYaw());
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		mChassisSpeeds = chassisSpeeds;
	}

	public void drive(SwerveModuleState[] states) {
		mChassisSpeeds = mKinematics.toChassisSpeeds(states);
	}

	PhotonCameraSubsystem pcw;

	@Override
	public void initialize() {
		// mPoseEstimator.resetPosition(new Pose2d(), new Rotation2d());
		zeroGyroscope();
		try {
			pcw = new PhotonCameraSubsystem();
		}
		catch (IOException e) {
			pcw = null;
		}
	}

	@Override
	public void periodic() {

		FluidicalPoseInfo poseInfo = mLimelight.getBotPose();

		//System.out.println(poseInfo);

		if (poseInfo != null && poseInfo.isValidTarget()) {
			mPoseEstimator.addVisionMeasurement(poseInfo.getPose(), poseInfo.getTimestamp());
		}

		// photonvision update pose

		// if(pcw.getEstimatedGlobalPose(mPoseEstimator.getEstimatedPosition()) != null) {
		// 	Optional<EstimatedRobotPose> result = pcw.getEstimatedGlobalPose(mPoseEstimator.getEstimatedPosition());
		// } else {
		// 	Optional<EstimatedRobotPose> result = //Set to a default value
		// }

		// if (result.isPresent()) {
		// 	EstimatedRobotPose camPose = result.get();
		// 	mPoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
		// 	System.out.println(camPose);
		// }

		mPoseEstimator.update(getGyroscopeRotation(), getPositions());

		SwerveModuleState[] states = mKinematics.toSwerveModuleStates(mChassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxVelocityMetersPerSecond);

		mFrontLeftModule.set(states[0].speedMetersPerSecond / kMaxVelocityMetersPerSecond * MAX_VOLTAGE, states[0].angle.getRadians());
		mFrontRightModule.set(states[1].speedMetersPerSecond / kMaxVelocityMetersPerSecond * MAX_VOLTAGE, states[1].angle.getRadians());
		mBackLeftModule.set(states[2].speedMetersPerSecond / kMaxVelocityMetersPerSecond * MAX_VOLTAGE, states[2].angle.getRadians());
		mBackRightModule.set(states[3].speedMetersPerSecond / kMaxVelocityMetersPerSecond * MAX_VOLTAGE, states[3].angle.getRadians());

	}

	public Pose2d getPose() {
		return mPoseEstimator.getEstimatedPosition();
	}

	public SwerveModulePosition[] getPositions() {
		return new SwerveModulePosition[] { new SwerveModulePosition(mFrontLeftModule.getPosition(), new Rotation2d(mFrontLeftModule.getSteerAngle())),
				new SwerveModulePosition(mFrontRightModule.getPosition(), new Rotation2d(mFrontRightModule.getSteerAngle())),
				new SwerveModulePosition(mBackLeftModule.getPosition(), new Rotation2d(mBackLeftModule.getSteerAngle())),
				new SwerveModulePosition(mBackRightModule.getPosition(), new Rotation2d(mBackRightModule.getSteerAngle())), };
	}

	public SwerveDriveKinematics getKinematics() {
		return mKinematics;
	}

	public Pose2d getWaypoint() {
		return mWaypoint;
	}

	public void setWaypoint(Pose2d waypoint) {
		mWaypoint = waypoint;
	}

	public LedPosition getLedPosition() {
		return mLedPosition;
	}

	public void setLedPosition(LedPosition position) {
		mLedPosition = position;
	}

	@Override
    public boolean checkSystem() {

        boolean isFunctional = false;
		double navxHeading = mNavx.getFusedHeading();

        if(navxHeading != 0.0) {
            isFunctional = false;
        } else {
			isFunctional = true;
        }
        return isFunctional;
    }

	@Override
	public void outputTelemetry() {

		

	}

	public synchronized static DriveSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new DriveSubsystem();
		}
		return mInstance;
	}
}
