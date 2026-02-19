package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.VisionConstants;

/**
 * Represents a standard AprilTag camera using PhotonVision.
 * Handles pose estimation, standard deviation calculations based on tag count/distance,
 * and simulation integration.
 */
public class AprilTagCamera {
    protected final String name;
    protected final PhotonCamera photonCamera;
    protected final PhotonPoseEstimator poseEstimator;
    protected PhotonCameraSim cameraSim; 
    public Matrix<N3, N1> curStdDevs;

    /**
     * Creates a new AprilTagCamera.
     *
     * @param name The name of the camera (must match PhotonVision pipeline name).
     * @param robotToCam The transform from the robot's center to the camera.
     * @param layout The AprilTag field layout.
     * @param visionSim The simulation system (can be null if not in sim).
     */
    public AprilTagCamera(String name, Transform3d robotToCam, AprilTagFieldLayout layout, VisionSystemSim visionSim) {
        this.name = name;
        this.photonCamera = new PhotonCamera(name);
        this.poseEstimator = new PhotonPoseEstimator(layout, robotToCam);
        this.curStdDevs = VecBuilder.fill(VisionConstants.SINGLE_TAG_STD_DEV, VisionConstants.SINGLE_TAG_STD_DEV, 2.0);

        if (RobotBase.isSimulation() && visionSim != null) {
            setupSimulation(visionSim, robotToCam);
        }
    }

    /**
     * Fetches the latest pipeline result.
     * In simulation, this pulls from the simulated camera instance.
     *
     * @return The latest PhotonPipelineResult.
     */
    public PhotonPipelineResult getLatestResult() {
        if (RobotBase.isSimulation() && cameraSim != null) {
            return cameraSim.getCamera().getAllUnreadResults().stream().reduce((first, second) -> second).orElse(new PhotonPipelineResult());
        }
        var results = photonCamera.getAllUnreadResults();
        return results.isEmpty() ? new PhotonPipelineResult() : results.get(results.size() - 1);
    }

    /**
     * Calculates the estimated robot pose based on visible AprilTags.
     *
     * @param prevPose The previous estimated pose (used for reference, though not always required by PhotonPoseEstimator).
     * @return An Optional containing the EstimatedRobotPose if a valid estimate is found.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevPose) {
        PhotonPipelineResult result = getLatestResult();
        if (!result.hasTargets()) return Optional.empty();

        double ambiguity = result.getBestTarget().getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity > VisionConstants.MAX_AMBIGUITY) return Optional.empty();

        Optional<EstimatedRobotPose> est = Optional.empty();
        var cameraMatrix = photonCamera.getCameraMatrix();
        var distCoeffs = photonCamera.getDistCoeffs();

        // High Accuracy Strategy
        if (cameraMatrix.isPresent() && distCoeffs.isPresent()) {
            est = poseEstimator.estimateRioMultiTagPose(result, cameraMatrix.get(), distCoeffs.get());
        } 
        
        // Fallback Strategy
        if (est.isEmpty()) {
            if (ambiguity == -1) return Optional.empty();
            est = poseEstimator.estimateLowestAmbiguityPose(result);
        }

        if (est.isPresent()) {
            updateEstimationStdDevs(est.get(), result.getTargets());
            return est;
        }

        return Optional.empty();
    }

    /**
     * Dynamically updates the standard deviations for the pose estimate based on the number and distance of tags.
     *
     * @param pose The estimated robot pose.
     * @param targets The list of visible targets used for the estimate.
     */
    protected void updateEstimationStdDevs(EstimatedRobotPose pose, List<PhotonTrackedTarget> targets) {
        Matrix<N3, N1> estStdDevs = VecBuilder.fill(VisionConstants.SINGLE_TAG_STD_DEV, VisionConstants.SINGLE_TAG_STD_DEV, 2.0);
        int numTags = 0;
        double avgDist = 0;

        for (var target : targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(pose.estimatedPose.toPose2d().getTranslation());
            numTags++;
        }

        if (numTags > 0) {
            avgDist /= numTags;
            if (numTags > 1) estStdDevs = VecBuilder.fill(VisionConstants.MULTI_TAG_STD_DEV, VisionConstants.MULTI_TAG_STD_DEV, 1.0);
            double scalar = (numTags == 1 && avgDist > 4) ? Double.MAX_VALUE : 1 + (avgDist * avgDist / 30.0);
            curStdDevs = estStdDevs.times(scalar);
        } else {
            curStdDevs = estStdDevs;
        }
    }

    /**
     * Configures the camera simulation properties.
     *
     * @param visionSim The VisionSystemSim instance.
     * @param robotToCam The camera's transform relative to the robot.
     */
    private void setupSimulation(VisionSystemSim visionSim, Transform3d robotToCam) {
        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(VisionConstants.SIM_RES_WIDTH, VisionConstants.SIM_RES_HEIGHT, Rotation2d.fromDegrees(VisionConstants.SIM_DIAG_FOV));
        props.setCalibError(VisionConstants.SIM_CALIB_ERROR_AVG, VisionConstants.SIM_CALIB_ERROR_STD_DEV);
        props.setFPS(VisionConstants.SIM_FPS);
        props.setAvgLatencyMs(VisionConstants.SIM_AVG_LATENCY_MS);
        props.setLatencyStdDevMs(VisionConstants.SIM_LATENCY_STD_DEV_MS);
        this.cameraSim = new PhotonCameraSim(photonCamera, props);
        visionSim.addCamera(cameraSim, robotToCam);
    }
}