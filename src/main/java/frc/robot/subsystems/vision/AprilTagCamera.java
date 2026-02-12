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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class AprilTagCamera {
    protected final String name;
    protected final PhotonCamera photonCamera;
    protected final PhotonPoseEstimator poseEstimator;
    protected PhotonCameraSim cameraSim; 
    public Matrix<N3, N1> curStdDevs;

    public AprilTagCamera(String name, Transform3d robotToCam, AprilTagFieldLayout layout, VisionSystemSim visionSim) {
        this.name = name;
        this.photonCamera = new PhotonCamera(name);
        this.poseEstimator = new PhotonPoseEstimator(layout, robotToCam);
        this.curStdDevs = VecBuilder.fill(VisionConstants.SINGLE_TAG_STD_DEV, VisionConstants.SINGLE_TAG_STD_DEV, 2.0);

        if (RobotBase.isSimulation() && visionSim != null) {
            setupSimulation(visionSim, robotToCam);
        }
    }

    public PhotonPipelineResult getLatestResult() {
        if (RobotBase.isSimulation() && cameraSim != null) {
            return cameraSim.getCamera().getAllUnreadResults().stream()
                .reduce((first, second) -> second).orElse(new PhotonPipelineResult());
        }
        var results = photonCamera.getAllUnreadResults();
        return results.isEmpty() ? new PhotonPipelineResult() : results.get(results.size() - 1);
    }

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
            if (ambiguity == -1) {
                 if (Timer.getFPGATimestamp() % 5.0 < 0.05) {
                     System.err.println("CRITICAL: [" + name + "] UNCALIBRATED & NO 3D DATA!");
                 }
                 return Optional.empty();
            }
            est = poseEstimator.estimateLowestAmbiguityPose(result);
        }

        if (est.isPresent()) {
            updateEstimationStdDevs(est.get(), result.getTargets());
            return est;
        }
        return Optional.empty();
    }

    protected void updateEstimationStdDevs(EstimatedRobotPose pose, List<PhotonTrackedTarget> targets) {
        Matrix<N3, N1> estStdDevs = VecBuilder.fill(VisionConstants.SINGLE_TAG_STD_DEV, VisionConstants.SINGLE_TAG_STD_DEV, 2.0);
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(pose.estimatedPose.toPose2d().getTranslation());
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