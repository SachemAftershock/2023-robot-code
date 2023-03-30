package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.lib.AftershockSubsystem;
import frc.lib.Limelight;
import frc.lib.PID;
import frc.lib.Limelight.FluidicalPoseInfo;
import frc.robot.ErrorTracker;
import frc.robot.ErrorTracker.ErrorType;
import frc.robot.enums.ButtonBoxLedInfo.LedPosition;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

	private boolean mWheelsLocked;

	private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

	private final Limelight mLimelight;

	private Pose2d mWaypoint;
	private LedPosition mLedPosition;

	private final PID mAntiTiltPID;
	private boolean mEnableBalance;
	private double antiTiltSpeed;

	private final boolean mEnableAntiTilt = false;

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
			tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
			Mk4SwerveModuleHelper.GearRatio.L1, kFrontRightDriveMotorId, kFrontRightSteerMotorId,
			kFrontRightSteerEncoderId, kFrontRightSteerOffset
		);

		mBackLeftModule = Mk4SwerveModuleHelper.createFalcon500Neo(
			tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
			Mk4SwerveModuleHelper.GearRatio.L1, kBackLeftDriveMotorId, kBackLeftSteerMotorId, kBackLeftSteerEncoderId,
			kBackLeftSteerOffset
		);

		mBackRightModule = Mk4SwerveModuleHelper.createFalcon500Neo(
			tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
			Mk4SwerveModuleHelper.GearRatio.L1, kBackRightDriveMotorId, kBackRightSteerMotorId,
			kBackRightSteerEncoderId, kBackRightSteerOffset
		);

		mFrontLeftModule.setCanStatusFramePeriodReductions();
		mFrontRightModule.setCanStatusFramePeriodReductions();
		mBackLeftModule.setCanStatusFramePeriodReductions();
		mBackRightModule.setCanStatusFramePeriodReductions();

		mPoseEstimator = new SwerveDrivePoseEstimator(mKinematics, new Rotation2d(), getPositions(), new Pose2d());

		mLimelight = new Limelight("limelight");

		mAntiTiltPID = new PID();
		mEnableBalance = true;
		antiTiltSpeed = 0.0;

		mWheelsLocked = false;
	}

	/**
	 * Sets the gyroscope angle to zero. This can be used to set the direction the
	 * robot is currently facing to the 'forwards' direction.
	 */
	public void zeroGyroscope() {
		mNavx.zeroYaw();
	}

	public double getYaw() {
		double yaw = mNavx.getYaw();

		ErrorTracker tracker = ErrorTracker.getInstance();
		if (yaw < 0.0001 && yaw > -0.0001) tracker.enableError(ErrorType.eNavXZero);
		else if (tracker.isErrorEnabled(ErrorType.eNavXZero)) tracker.disableError(ErrorType.eNavXZero);

		return yaw;
	}

	public double getAdjustedYaw() {
		return (getYaw());
	}

	public Rotation2d getGyroscopeRotation() {
		if (mNavx.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(mNavx.getFusedHeading());
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - (getYaw())); // TODO: Add 90 here I think
	}

	public void drive(ChassisSpeeds chassisSpeeds) {
		mChassisSpeeds = chassisSpeeds;
	}

	public void drive(SwerveModuleState[] states) {
		mChassisSpeeds = mKinematics.toChassisSpeeds(states);
	}

	//instead of using set<oduleStates use the method above
	PhotonCameraSubsystem pcw;

	@Override
	public void initialize() {
		zeroGyroscope();
		mPoseEstimator.resetPosition(getGyroscopeRotation(), getPositions(), new Pose2d());
		try {
			pcw = new PhotonCameraSubsystem();
		}
		catch (IOException e) {
			pcw = null;
		}

		drive(new ChassisSpeeds());
	}

	@Override
	public void periodic() {

		FluidicalPoseInfo poseInfo = mLimelight.getBotPose();

		// System.out.println(poseInfo);

		if (poseInfo != null && poseInfo.isValidTarget()) {
			mPoseEstimator.addVisionMeasurement(poseInfo.getPose(), poseInfo.getTimestamp());
		}

		// System.out.println("Angle --> " + mNavx.getYaw());

		// photonvision update pose

		// if(pcw.getEstimatedGlobalPose(mPoseEstimator.getEstimatedPosition()) != null)
		// {
		// Optional<EstimatedRobotPose> result =
		// pcw.getEstimatedGlobalPose(mPoseEstimator.getEstimatedPosition());
		// } else {
		// Optional<EstimatedRobotPose> result = //Set to a default value
		// }

		// if (result.isPresent()) {
		// EstimatedRobotPose camPose = result.get();
		// mPoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
		// camPose.timestampSeconds);
		// System.out.println(camPose);
		// }

		mPoseEstimator.update(getGyroscopeRotation(), getPositions());

		//System.out.println(mPoseEstimator.getEstimatedPosition());

		if (mWheelsLocked) {
			double angle = Math.toRadians(45);

			mFrontLeftModule.set(0, angle);
			mFrontRightModule.set(0, -angle);
			mBackLeftModule.set(0, -angle);
			mBackRightModule.set(0, angle);
			return;
		}

		SwerveModuleState[] states = mKinematics.toSwerveModuleStates(mChassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(states, kManualMaxVelocityMetersPerSecond);

		mFrontLeftModule.set(
			states[0].speedMetersPerSecond / kManualMaxVelocityMetersPerSecond * MAX_VOLTAGE,
			states[0].angle.getRadians()
		);
		mFrontRightModule.set(
			states[1].speedMetersPerSecond / kManualMaxVelocityMetersPerSecond * MAX_VOLTAGE,
			states[1].angle.getRadians()
		);
		mBackLeftModule.set(
			states[2].speedMetersPerSecond / kManualMaxVelocityMetersPerSecond * MAX_VOLTAGE,
			states[2].angle.getRadians()
		);
		mBackRightModule.set(
			states[3].speedMetersPerSecond / kManualMaxVelocityMetersPerSecond * MAX_VOLTAGE,
			states[3].angle.getRadians()
		);

	}

	public Pose2d getPose() {
		return mPoseEstimator.getEstimatedPosition();
	}

	public SwerveModulePosition[] getPositions() {
		return new SwerveModulePosition[] { new SwerveModulePosition(
			mFrontLeftModule.getPosition(), new Rotation2d(mFrontLeftModule.getSteerAngle())
		), new SwerveModulePosition(mFrontRightModule.getPosition(), new Rotation2d(mFrontRightModule.getSteerAngle())),
				new SwerveModulePosition(
					mBackLeftModule.getPosition(), new Rotation2d(mBackLeftModule.getSteerAngle())
				),
				new SwerveModulePosition(
					mBackRightModule.getPosition(), new Rotation2d(mBackRightModule.getSteerAngle())
				), };
	}

	public double[] runAntiTiltControl(double powX, double powY) {
        double robotPitch = mNavx.getPitch();
        double robotRoll = mNavx.getRoll();
        double[] NewPowXY = new double[2];
        NewPowXY[0] = powY;
        NewPowXY[1] = powX;

        if ( (mEnableAntiTilt) && (Math.abs(robotPitch) > kMinTiltAngle) && (Math.abs(robotPitch) < kMaxTiltAngle) ) {
            double slope = (kTiltSlope - 0.0) / (kMaxTiltAngle - kMinTiltAngle);
            double correctionOffset = slope * (robotPitch - kMinTiltAngle);
            NewPowXY[0] += correctionOffset;
            System.out.println("ERROR : Anti-Tilt Control Active " + correctionOffset + " Pitch: " + robotPitch);
        }

        if ( (mEnableAntiTilt) && (Math.abs(robotRoll) > kMinTiltAngle) && (Math.abs(robotRoll) < kMaxTiltAngle) ) {
            double slope = (kTiltSlope - 0.0) / (kMaxTiltAngle - kMinTiltAngle);
            double correctionOffset = slope * (robotRoll - kMinTiltAngle);
            NewPowXY[1] += correctionOffset;
            System.out.println("ERROR : Anti-Tilt Control Active " + correctionOffset + " Yaw: " + robotRoll);
        }
        
        return NewPowXY;
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

	/*
	 * @return The robot relative pitch
	 */
	public double getPitch() {
		return mNavx.getRoll();
	}

	/*
	 * @return The robot relative roll
	 */
	public double getRoll() {
		return mNavx.getPitch();
	}

	public double[] runBalanceControl(double pow, double rot) {
		double robotPitch = mNavx.getPitch();
		double NewPowRot[] = new double[2];
		NewPowRot[0] = pow;
		NewPowRot[1] = rot;

		if ((mEnableBalance) && (Math.abs(robotPitch) > kMinBalanceAngle)
			&& (Math.abs(robotPitch) < kMaxBalanceAngle)) {
			double slope = (0.4 - 0.0) / (kMaxBalanceAngle - kMinBalanceAngle);
			double correctionOffset = slope * (robotPitch - kMinBalanceAngle);
			NewPowRot[0] = NewPowRot[0] + correctionOffset;
			NewPowRot[1] = -rot;
			// System.out.println("ERROR : Anti-Tilt Control Active " + correctionOffset + "
			// Pitch :" + robotPitch);
		}
		return NewPowRot;
	}
	// TODO I wasn't sure which code was the correct to merge and
	// TODO mohid wasn't here to ask so I just merged both
	// double robotPitch = mNavx.getPitch();
	// double robotRoll = mNavx.getRoll();

	// double NewPowRot[] = new double[2];
	// NewPowRot[0] = pow;
	// NewPowRot[1] = rot;

	// if ( (mEnableBalance) && (Math.abs(robotPitch) > kMinBalanceAngle) &&
	// (Math.abs(robotPitch) < kMaxBalanceAngle) ) {
	// double slope = (0.4 - 0.0) / (kMaxBalanceAngle - kMinBalanceAngle);
	// double correctionOffset = slope * (robotPitch - kMinBalanceAngle);
	// NewPowRot[0] = -(NewPowRot[0] + correctionOffset);
	// // System.out.println("ERROR : Anti-Tilt Control Active " + correctionOffset
	// + "
	// // Pitch :" + robotPitch);
	// }
	// else if ( (mEnableBalance) && (Math.abs(robotRoll) > kMinBalanceAngle) &&
	// (Math.abs(robotRoll) < kMaxBalanceAngle) ) {
	// double slope = (0.4 - 0.0) / (kMaxBalanceAngle - kMinBalanceAngle);
	// double correctionOffset = slope * (robotRoll - kMinBalanceAngle);
	// NewPowRot[0] = -(NewPowRot[0] + correctionOffset);
	// // System.out.println("ERROR : Anti-Tilt Control Active " + correctionOffset
	// + "
	// // Pitch :" + robotPitch);
	// }
	// return NewPowRot;
	// }

	public double balanceOnChargeStation() {
		double currentTiltAngle = getChargeStationTiltAngle();
		double tiltError = kTargetBalanceAngle - currentTiltAngle;

		if (Math.abs(tiltError) < kBalanceThreshold) {
			return 0;
		}
		else {
			double tiltDirection = Math.signum(tiltError);
			return (kDriveSpeed * tiltDirection);
		}
	}

	public double getChargeStationTiltAngle() {
		if (getRoll() > 3) {
			return getRoll();
		}
		else if (getPitch() > 3) {
			return getPitch();
		}
		return 0; // replace with actual measurement
	}

	public double holdPosition() {
		double setpoint = 0; // get the current angle as the setpoint

		// PID constants
		double kP = 0.01;
		double kI = 0.0;
		double kD = 0.0;

		// PID variables
		double error, integral = 0, derivative;
		double lastError = 0;

		// loop until interrupted or disabled
		// calculate error and update PID variables
		error = setpoint - getChargeStationTiltAngle();
		integral += error;
		derivative = error - lastError;

		// calculate output using PID equation
		double output = kP * error + kI * integral + kD * derivative;

		// set the motor output to hold the position

		// update last error
		lastError = error;

		return output;
	}

	public double[] runAntiTilt(double pow) {
		double robotPitch = mNavx.getPitch();
		double robotRoll = mNavx.getRoll();
		double NewPowRot[] = new double[2];
		NewPowRot[0] = pow;

		if ((mEnableBalance) && (Math.abs(robotPitch) > kMinBalanceAngle)
			&& (Math.abs(robotPitch) < kMaxBalanceAngle)) {
			double slope = (0.4 - 0.0) / (kMaxBalanceAngle - kMinBalanceAngle);
			double correctionOffset = slope * (robotPitch - kMinBalanceAngle);
			NewPowRot[0] = (NewPowRot[0] + correctionOffset);
			// System.out.println("ERROR : Anti-Tilt Control Active " + correctionOffset + "
			// Pitch :" + robotPitch);
		}
		else if ((mEnableBalance) && (Math.abs(robotRoll) > kMinBalanceAngle)
			&& (Math.abs(robotRoll) < kMaxBalanceAngle)) {
				double slope = (0.4 - 0.0) / (kMaxBalanceAngle - kMinBalanceAngle);
				double correctionOffset = slope * (robotRoll - kMinBalanceAngle);
				NewPowRot[0] = (NewPowRot[0] + correctionOffset);
				// System.out.println("ERROR : Anti-Tilt Control Active " + correctionOffset + "
				// Pitch :" + robotPitch);
			}
		return NewPowRot;
	}

	public double holdGround() {
		double robotPitch = mNavx.getPitch();
		double robotRoll = mNavx.getRoll();

		return -1;
	}

	public void lockWheels() {
		mWheelsLocked = true;
	}

	public void unlockWheels() {
		mWheelsLocked = false;
	}

	@Override
	public boolean checkSystem() {

		boolean isFunctional = false;
		double navxHeading = mNavx.getFusedHeading();

		if (navxHeading != 0.0) {
			isFunctional = false;
		}
		else {
			isFunctional = true;
		}
		return isFunctional;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putNumber("Robot Pitch ", mNavx.getPitch());
	}

	public synchronized static DriveSubsystem getInstance() {
		if (mInstance == null) {
			mInstance = new DriveSubsystem();
		}
		return mInstance;
	}
	public void resetOdometry(PathPlannerTrajectory trag){
		mPoseEstimator.resetPosition(trag.getInitialHolonomicPose().getRotation(), getPositions(), trag.getInitialHolonomicPose());
	}

	//DO NOT USE
	public Command followPathTrajectoryRed(boolean isFirstPath, PathPlannerTrajectory traj){
		return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj);
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
			this::getPose, // Pose supplier
            getKinematics(), // SwerveDriveKinematics
            new PIDController(4.0/*-12.5*/, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(4.0/*12.5*/, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(10, kDriveAngularGains[1], kDriveAngularGains[2]), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::drive, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
	}
	public Command followPathTrajectoryBlue(boolean isFirstPath, PathPlannerTrajectory traj){
		return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj);
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
			this::getPose, // Pose supplier
            getKinematics(), // SwerveDriveKinematics
            new PIDController(-7.95/*-12.5*/, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(7.95/*12.5*/, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(10, kDriveAngularGains[1], kDriveAngularGains[2]), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::drive, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
	}
	public Command followPathTrajectoryXLinear(boolean isFirstPath, PathPlannerTrajectory traj){
		return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj);
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
			this::getPose, // Pose supplier
            getKinematics(), // SwerveDriveKinematics
            new PIDController(-7.95/*-12.5*/, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(10, kDriveAngularGains[1], kDriveAngularGains[2]), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::drive, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
	}
	public Command followPathTrajectoryYLinear(boolean isFirstPath, PathPlannerTrajectory traj){
		return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj);
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
			this::getPose, // Pose supplier
            getKinematics(), // SwerveDriveKinematics
            new PIDController(0/*-12.5*/, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(7.95, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(10, kDriveAngularGains[1], kDriveAngularGains[2]), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::drive, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
	}
	
}
