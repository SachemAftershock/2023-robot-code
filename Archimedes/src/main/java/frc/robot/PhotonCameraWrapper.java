package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;


public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    public PhotonPoseEstimator photonPoseEstimator;

    //todo: why io exception? is it needed
    public PhotonCameraWrapper() throws IOException {
        
        //Todo: check if layout works
        final AprilTagFieldLayout atfl =  new AprilTagFieldLayout("src/main/java/frc/robot/2023-chargedup-atf1.json");
        
        //todo: add camera vision name to camera name in photonvision UI
        //makes photoncamera object
        photonCamera = new PhotonCamera(VisionConstants.cameraName); 
        
        // Create pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, VisionConstants.robotToCam);
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();

    }
    public void photonPeriodic(){
        /* 
        Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(Drivesubsystem.mPoseEstimator.getEstimatedPosition());
        //get pose
        EstimatedRobotPose pose = getEstimatedGlobalPose(DriveSubsystem.mPoseEstimator.getEstimatedPosition());
    
        //get pose.estimatedPose and pose.timestampSeconds
        //estimatedpose puts in a 3d pose? we need 2d?
        
		
        //todo: do i need to check if it has targets first?

        if (pose.isPresent) {
        m_poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        }
        */

   }
 }
