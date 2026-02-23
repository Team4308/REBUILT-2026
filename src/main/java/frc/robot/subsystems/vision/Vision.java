package frc.robot.subsystems.vision;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConfig.CameraConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.VisionSystemSim;

/**
 * The Vision subsystem coordinates all camera operations, including AprilTag-based
 * pose estimation and object detection. It manages the lifecycle of individual
 * camera objects and aggregates their data to update the robot's pose estimator.
 */
public class Vision extends SubsystemBase {
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final Map<String, ObjectDetectionCamera> objectCameras = new HashMap<>();
    private final VisionSystemSim visionSim;
    private final SwerveSubsystem swerve; 

    public final List<AprilTagCamera> tagCameras = new ArrayList<>();
    public record TargetData(Rotation2d angle, double distance) {}

    /**
     * Constructs the Vision subsystem.
     * Initializes the simulation environment (if applicable) and loads camera configurations.
     *
     * @param swerve The SwerveSubsystem instance used for pose estimation updates.
     */
    public Vision(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.visionSim = RobotBase.isSimulation() ? new VisionSystemSim("Vision") : null;
        if (visionSim != null && fieldLayout != null) visionSim.addAprilTags(fieldLayout);

        loadCameraConfigs();
        
        System.out.println("---------- VISION SUBSYSTEM INITIALIZED ----------");
    }

    /**
     * Loads camera configurations from the "vision-config.json" file located in the deploy directory.
     * Instantiates {@link AprilTagCamera} or {@link ObjectDetectionCamera} objects based on the config.
     */
    private void loadCameraConfigs() {
        File configFile = new File(Filesystem.getDeployDirectory(), "vision-config.json");
        if (!configFile.exists()) DriverStation.reportWarning("Critical: Camera config file not found", false);

        try {
            ObjectMapper mapper = new ObjectMapper();
            VisionConfig config = mapper.readValue(configFile, VisionConfig.class);

            for (CameraConfig cam : config.cameras) {
                if (!cam.enabled) continue;

                NetworkTableInstance.getDefault().getTable("Robot/Vision").getSubTable(cam.name).getStringTopic("Mode").publish().set(cam.streamType);

                boolean isEx = "PYTHON".equalsIgnoreCase(cam.streamType);
                if ("APRILTAG".equals(cam.type) && fieldLayout != null) {
                    tagCameras.add(isEx ? new ExperimentalAprilTagCamera(cam.name, cam.transform.toTransform3d(), fieldLayout, visionSim) : new AprilTagCamera(cam.name, cam.transform.toTransform3d(), fieldLayout, visionSim));
                } else if ("OBJECT_DETECTION".equals(cam.type)) {
                    objectCameras.put(cam.name, isEx ? new ExperimentalObjectDetectionCamera(cam.name) : new ObjectDetectionCamera(cam.name));
                }

                System.out.println("Vision: Loaded " + cam.name + " (" + cam.streamType + ")");
            }

        } catch (IOException e) { 
            e.printStackTrace(); 
        }
    }

    /**
     * Collects pose estimates from all active AprilTag cameras and updates the SwerveSubsystem.
     * Also updates the simulation physics if running in a simulation environment.
     */
    private void updatePoseEstimation() {
        Pose2d currentPose = swerve.getPose();
        
        // Sim Logic
        if (RobotBase.isSimulation() && visionSim != null) {
            if (swerve.getSwerveDrive().getSimulationDriveTrainPose().isPresent()) {
                visionSim.update(swerve.getSwerveDrive().getSimulationDriveTrainPose().get());
            }
        }

        for (AprilTagCamera cam : tagCameras) {
            var poseEst = cam.getEstimatedGlobalPose(currentPose);
            
            if (poseEst.isPresent()) {
                var pose = poseEst.get();
                swerve.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(), 
                    pose.timestampSeconds, 
                    cam.curStdDevs
                );
            }
        }
    }

    @Override
    public void periodic() {
        updatePoseEstimation();
    }

    /**
     * Estimates distance to an object detection target.
     * Must be calibrated; current implementation relies on an inverse square root of target area as a placeholder.
     * For high accuracy, substitute with pitch-based trigonometry using camera and target height.
     * * @param target The tracked target.
     * @return Estimated distance.
     */
    private double estimateDistance(PhotonTrackedTarget target) {
        if (target.getArea() <= 0) return Double.MAX_VALUE;
        // Placeholder constant (10.0) needs tuning based on your specific camera and object scale.
        return 10.0 / Math.sqrt(target.getArea()); 
    }

    /**
     * Retrieves the angles and estimated distances of all targets detected by a specific object detection camera.
     *
     * @param cameraName The name of the camera to query.
     * @return A list of TargetData containing the yaw angle and estimated distance to each target.
     */
    public List<TargetData> getTargetAngles(String cameraName) {
        ObjectDetectionCamera cam = objectCameras.get(cameraName);
        List<TargetData> targetsInfo = new ArrayList<>();
        if (cam != null) {
            var result = cam.getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) {
                    targetsInfo.add(new TargetData(Rotation2d.fromDegrees(target.getYaw()), estimateDistance(target)));
                }
            }
        }
        return targetsInfo;
    }

    /**
     * Retrieves the best (highest confidence/largest area) target's angle and estimated distance from a specific camera.
     *
     * @param cameraName The name of the camera to query.
     * @return An Optional containing the TargetData for the best target if one exists, otherwise empty.
     */
    public Optional<TargetData> getBestTarget(String cameraName) {
        ObjectDetectionCamera cam = objectCameras.get(cameraName);
        if (cam != null) {
            Optional<PhotonTrackedTarget> targetOpt = cam.getBestTarget();
            if (targetOpt.isPresent()) {
                PhotonTrackedTarget target = targetOpt.get();
                return Optional.of(new TargetData(Rotation2d.fromDegrees(target.getYaw()), estimateDistance(target)));
            }
        }
        return Optional.empty();
    }

    /**
     * Evaluates all visible targets and retrieves the one with the shortest estimated distance.
     *
     * @param cameraName The name of the camera to query.
     * @return An Optional containing the TargetData for the closest target if one exists, otherwise empty.
     */
    public Optional<TargetData> getClosestTarget(String cameraName) {
        ObjectDetectionCamera cam = objectCameras.get(cameraName);
        if (cam == null) return Optional.empty();

        var result = cam.getLatestResult();
        if (!result.hasTargets()) return Optional.empty();

        PhotonTrackedTarget closestTarget = null;
        double minDistance = Double.MAX_VALUE;

        for (PhotonTrackedTarget target : result.getTargets()) {
            double distance = estimateDistance(target);
            if (distance < minDistance) {
                minDistance = distance;
                closestTarget = target;
            }
        }

        if (closestTarget != null) {
            return Optional.of(new TargetData(Rotation2d.fromDegrees(closestTarget.getYaw()), minDistance));
        }

        return Optional.empty();
    }
}