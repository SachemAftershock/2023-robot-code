package frc.lib;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Wrapper class for getting and setting Limelight NetworkTable values.
 * 
 * @author Dan Waxman
 * @author Shreyas Prasad
 * @author Ty McKeon
*/
public class Limelight {
	
	private NetworkTableInstance table = null;

	private final String mTableName;

	public final static double kDefaultValue = 9999.9;
	
	/**
	 * Creates a new Limelight Object
	 * @param tableName The name of the Limelight's NetworkTable
	 */
	public Limelight(String tableName) {
		mTableName = tableName;
	}

	/**
	 * Light modes for Limelight.
	 * 
	 * @author Dan Waxman
	 */
	public static enum LightMode {
		ePipeline, eOff, eBlink, eOn
	}

	/**
	 * Camera modes for Limelight.
	 * 
	 * @author Dan Waxman
	 */
	public static enum CameraMode {
		eVision, eDriver
	}

	/**
	 * Gets whether a target is detected by the Limelight.
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return true if a target is detected, false otherwise.
	 */
	public boolean isTarget() {
		return getValue("tv").getDouble(kDefaultValue) == 1;
	}

	/**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		return getValue("tx").getDouble(kDefaultValue);
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return ty as reported by the Limelight.
	 */
	public double getTy() {
		return getValue("ty").getDouble(kDefaultValue);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return Area of target.
	 */
	public double getTa() {
		return getValue("ta").getDouble(kDefaultValue);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return Target skew.
	 */
	public double getTs() {
		return getValue("ts").getDouble(kDefaultValue);
	}

	/**
	 * Gets target latency (ms).
	 * <p>
	 * Default value is 9999.9
	 * 
	 * @return Target latency.
	 */
	public double getTl() {
		return getValue("tl").getDouble(kDefaultValue);
	}


/**
	 * 
	 * Gets the robots pose in field space space as computed by solvepnp (x,y,z,rx,ry,rz).
	 * <p>
	 * Will return null if no targets are found.
	 * <p>
	 * Requires a third part JSON library https://mvnrepository.com/artifact/org.json/json
	 * 
	 * @return Matrix of Robot Pose in field space for each target
	 * <p>
	 * <li> Null if no targets found
	 */
	public FluidicalPoseInfo getBotPose() {
		try {
			String rawJSON = getValue("json").getString(""); //get the JSON dump from NetworkTables
			if (rawJSON.isEmpty()) return null;

			JSONObject json = new JSONObject(rawJSON);
			JSONObject jsonResults = json.getJSONObject("Results");
			boolean validTarget = jsonResults.getInt("v") == 1;
			double timestamp = jsonResults.getDouble("ts");

			Alliance alliance = DriverStation.getAlliance();
			String allianceSidePose;

			if(alliance == Alliance.Blue) {
				allianceSidePose = "botpose_wpiblue";
			} else if(alliance == Alliance.Red) {
				allianceSidePose = "botpose_wpired";
			} else {
				throw new Exception("Alliance not Found!");
			}

			JSONArray poseArray = jsonResults.getJSONArray(allianceSidePose);

			Pose2d fieldRelativePose = new Pose2d(
					poseArray.getDouble(0), poseArray.getDouble(1),
					new Rotation2d(poseArray.getDouble(3), poseArray.getDouble(4))
			);
	
			return new FluidicalPoseInfo(fieldRelativePose, validTarget, timestamp);
		} catch (Exception e) {
			DriverStation.reportError("Grave error with limelight parsing", e.getStackTrace());
		}
		
		return null;

	}

	public FluidicalPoseInfo getBotPoseViaNetworkTables() {
		
		try {
			Alliance alliance = DriverStation.getAlliance();
			if (alliance == Alliance.Invalid) throw new Exception("Alliance not Found!");

			String allianceSidePose = alliance == Alliance.Blue ? "botpose_wpiblue" : "botpose_wpired";
			double[] result = getValue(allianceSidePose).getDoubleArray(new double[]{});

			double ts = getValue("ts").getDouble(-1);
			int tv = (int) getValue("tv").getInteger(0l);
			
			if (ts == -1) throw new Exception("No timestamp found");
			if (result.length == 0) throw new Error("No targets found");

			boolean isValid = tv == 1;

			Pose2d fieldRelativePose = new Pose2d(
				result[0], result[1], 
				new Rotation2d(result[3], result[4])
			);

			return new FluidicalPoseInfo(fieldRelativePose, isValid, ts);
		} catch (Exception e) {
			DriverStation.reportError("Gave error with Limelight Pose", e.getStackTrace());
		}
		
		return null;
	}

	/**
	 * Sets LED mode of Limelight.
	 * 
	 * @param mode
	 *            Light mode for Limelight.
	 */
	public void setLedMode(LightMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets camera mode for Limelight.
	 * 
	 * @param mode
	 *            Camera mode for Limelight.
	 */
	public void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets pipeline number (0-9 value).
	 * 
	 * @param number
	 *            Pipeline number (0-9).
	 */
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}

	/**
	 * Helper method to get an entry from the Limelight NetworkTable.
	 * 
	 * @param key
	 *            Key for entry.
	 * @return NetworkTableEntry of given entry.
	 */
	private NetworkTableEntry getValue(String key) {
		if (table == null) {
			table = NetworkTableInstance.getDefault();
		}
		return table.getTable(mTableName).getEntry(key);
	}

	public void outputTelemetry() {
		SmartDashboard.putBoolean(mTableName + " is Target", isTarget());
		SmartDashboard.putNumber(mTableName + " tx", getTx());
		SmartDashboard.putNumber(mTableName + " ty", getTy());
		SmartDashboard.putNumber(mTableName + " ta", getTa());
		SmartDashboard.putNumber(mTableName + " ts", getTs());
		SmartDashboard.putNumber(mTableName + " tl", getTl());
	}

	public class FluidicalPoseInfo {

		private Pose2d mPose2d;
		private boolean mValidTarget;
		private double mTimestampSeconds;

		public FluidicalPoseInfo(Pose2d pose, boolean validTarget, double timestampMs) {
			mPose2d = pose;
			mValidTarget = validTarget;
			mTimestampSeconds = timestampMs/1000.0; //converting time to seconds
		}

		public double getTimestamp() {
			return mTimestampSeconds;
		}

		public Pose2d getPose() {
			return mPose2d;
		}

		public boolean isValidTarget() {
			return mValidTarget;
		}

		@Override
		public String toString() {
			String prefix = mValidTarget ? "" : "[X]";
			return prefix + mTimestampSeconds + ": " + mPose2d.toString();
		}
	}
}