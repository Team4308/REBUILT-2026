package frc.robot.Subsystems.vision;

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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.vision.VisionConfig.CameraConfig;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.VisionSystemSim;

/**
 * The Vision subsystem coordinates all camera operations, including
 * AprilTag-based
 * pose estimation and object detection. It acts as a standalone data provider.
 */
public class Vision extends SubsystemBase {

    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public final List<AprilTagCamera> tagCameras = new ArrayList<>();

    private final Map<String, ObjectDetectionCamera> objectCameras = new HashMap<>();
    private final Map<String, CameraConfig> cameraConfigs = new HashMap<>();
    private final Map<String, Double> targetWidths = new HashMap<>();

    private final VisionSystemSim visionSim;

    /**
     * A record holding a detected object's calculated angle and estimated distance.
     */
    public record ObjectData(Rotation2d angle, double distance) {
    }

    /**
     * A record holding an estimated global pose and its calculated standard
     * deviations.
     */
    public record VisionMeasurement(EstimatedRobotPose estimation, Matrix<N3, N1> stdDevs) {
    }

    /**
     * Constructs the Vision subsystem.
     * Initializes the simulation environment (if applicable) and loads camera
     * configurations.
     */
    public Vision() {
        this.visionSim = RobotBase.isSimulation() ? new VisionSystemSim("Vision") : null;
        if (visionSim != null && fieldLayout != null)
            visionSim.addAprilTags(fieldLayout);

        loadCameraConfigs();

        System.out.println("---------- VISION SUBSYSTEM INITIALIZED ----------");
    }

    private void loadCameraConfigs() {
        File configFile = new File(Filesystem.getDeployDirectory(), "vision-config.json");
        if (!configFile.exists())
            DriverStation.reportWarning("Critical: Camera config file not found", false);

        try {
            ObjectMapper mapper = new ObjectMapper();
            VisionConfig config = mapper.readValue(configFile, VisionConfig.class);

            if (config.targetWidths != null) {
                targetWidths.putAll(config.targetWidths);
            }

            for (CameraConfig cam : config.cameras) {
                cameraConfigs.put(cam.name, cam);

                if (!cam.enabled)
                    continue;

                NetworkTableInstance.getDefault().getTable("Robot/Vision").getSubTable(cam.name).getStringTopic("Mode")
                        .publish().set(cam.streamType);

                boolean isEx = "PYTHON".equalsIgnoreCase(cam.streamType);
                if ("APRILTAG".equals(cam.type) && fieldLayout != null) {
                    tagCameras.add(isEx
                            ? new ExperimentalAprilTagCamera(cam.name, cam.transform.toTransform3d(), fieldLayout,
                                    visionSim)
                            : new AprilTagCamera(cam.name, cam.transform.toTransform3d(), fieldLayout, visionSim));
                } else if ("OBJECT_DETECTION".equals(cam.type)) {
                    objectCameras.put(cam.name, isEx ? new ExperimentalObjectDetectionCamera(cam.name)
                            : new ObjectDetectionCamera(cam.name));
                }

                System.out.println("Vision: Loaded " + cam.name + " (" + cam.streamType + ")");
            }

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Generates pose estimates from all active AprilTag cameras.
     * * @param currentPose The robot's current estimated pose (used for ambiguity
     * resolution).
     * 
     * @return A list of valid VisionMeasurements.
     */
    public List<VisionMeasurement> getVisionMeasurements(Pose2d currentPose) {
        List<VisionMeasurement> measurements = new ArrayList<>();
        for (AprilTagCamera cam : tagCameras) {
            Optional<EstimatedRobotPose> poseEst = cam.getEstimatedGlobalPose(currentPose);
            if (poseEst.isPresent()) {
                measurements.add(new VisionMeasurement(poseEst.get(), cam.curStdDevs));
            }
        }
        return measurements;
    }

    /**
     * Updates the vision simulation environment with the robot's actual driven
     * pose.
     * * @param robotPose The true robot pose in the simulation.
     */
    public void updateSimVision(Pose2d robotPose) {
        if (RobotBase.isSimulation() && visionSim != null && robotPose != null) {
            visionSim.update(robotPose);
        }
    }

    private double getObjectWidthPixels(PhotonTrackedTarget objectTarget) {
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;

        var corners = objectTarget.getDetectedCorners();
        if (corners == null || corners.isEmpty())
            return 0.0;

        for (var corner : corners) {
            if (corner.x < minX)
                minX = corner.x;
            if (corner.x > maxX)
                maxX = corner.x;
        }
        return maxX - minX;
    }

    private double estimateDistance(PhotonTrackedTarget objectTarget, String cameraName) {
        CameraConfig config = cameraConfigs.get(cameraName);
        if (config == null || config.resolution == null)
            return Double.MAX_VALUE;

        double widthPixels = getObjectWidthPixels(objectTarget);
        if (widthPixels <= 0)
            return Double.MAX_VALUE;

        String objectClass = String.valueOf(objectTarget.getFiducialId());
        double gamepieceDiameterMeters = targetWidths.getOrDefault(objectClass, 0.3556);

        double cameraFovRads = Math.toRadians(config.resolution.fov);
        double pixelToRad = config.resolution.width / cameraFovRads;
        double thetaRads = widthPixels / pixelToRad;

        return gamepieceDiameterMeters / Math.tan(thetaRads);
    }

    public List<ObjectData> getObjectAngles(String cameraName) {
        ObjectDetectionCamera cam = objectCameras.get(cameraName);
        List<ObjectData> objectsInfo = new ArrayList<>();
        if (cam != null) {
            var result = cam.getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget objectTarget : result.getTargets()) {
                    objectsInfo.add(new ObjectData(Rotation2d.fromDegrees(objectTarget.getYaw()),
                            estimateDistance(objectTarget, cameraName)));
                }
            }
        }
        return objectsInfo;
    }

    public Optional<ObjectData> getBestObject(String cameraName) {
        ObjectDetectionCamera cam = objectCameras.get(cameraName);
        if (cam != null) {
            Optional<PhotonTrackedTarget> targetOpt = cam.getBestTarget();
            if (targetOpt.isPresent()) {
                PhotonTrackedTarget objectTarget = targetOpt.get();
                return Optional.of(new ObjectData(Rotation2d.fromDegrees(objectTarget.getYaw()),
                        estimateDistance(objectTarget, cameraName)));
            }
        }
        return Optional.empty();
    }

    public Optional<ObjectData> getClosestObject(String cameraName) {
        ObjectDetectionCamera cam = objectCameras.get(cameraName);
        if (cam == null)
            return Optional.empty();

        var result = cam.getLatestResult();
        if (!result.hasTargets())
            return Optional.empty();

        PhotonTrackedTarget closestObject = null;
        double minDistance = Double.MAX_VALUE;

        for (PhotonTrackedTarget objectTarget : result.getTargets()) {
            double distance = estimateDistance(objectTarget, cameraName);
            if (distance < minDistance) {
                minDistance = distance;
                closestObject = objectTarget;
            }
        }

        if (closestObject != null) {
            return Optional.of(new ObjectData(Rotation2d.fromDegrees(closestObject.getYaw()), minDistance));
        }

        return Optional.empty();
    }
}