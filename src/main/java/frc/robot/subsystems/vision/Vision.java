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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConfig.CameraConfig;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.VisionSystemSim;

public class Vision extends SubsystemBase {

    private final AprilTagFieldLayout fieldLayout;
    public final List<AprilTagCamera> tagCameras = new ArrayList<>();
    private final Map<String, ObjectDetectionCamera> objectCameras = new HashMap<>();
    
    private final VisionSystemSim visionSim;
    private final SwerveSubsystem swerve; 

    public Vision(SwerveSubsystem swerve) {
        this.swerve = swerve;

        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            layout = null; 
            System.err.println("CRITICAL: Field Layout failed to load.");
        }
        this.fieldLayout = layout;

        this.visionSim = RobotBase.isSimulation() ? new VisionSystemSim("Vision") : null;
        if (visionSim != null && fieldLayout != null) visionSim.addAprilTags(fieldLayout);

        loadCameraConfigs();
        System.out.println("---------- VISION SUBSYSTEM INITIALIZED ----------");
    }

    private void loadCameraConfigs() {
        File configFile = new File(Filesystem.getDeployDirectory(), "vision-config.json");
        if (!configFile.exists()) return;

        try {
            ObjectMapper mapper = new ObjectMapper();
            VisionConfig config = mapper.readValue(configFile, VisionConfig.class);

            for (CameraConfig cam : config.cameras) {
                if (!cam.enabled) continue;

                NetworkTableInstance.getDefault().getTable("Robot/Vision").getSubTable(cam.name)
                    .getStringTopic("Mode").publish().set(cam.streamType);

                boolean isEx = "PYTHON".equalsIgnoreCase(cam.streamType);
                if ("APRILTAG".equals(cam.type) && fieldLayout != null) {
                    tagCameras.add(isEx ? new ExperimentalAprilTagCamera(cam.name, cam.transform.toTransform3d(), fieldLayout, visionSim) 
                                        : new AprilTagCamera(cam.name, cam.transform.toTransform3d(), fieldLayout, visionSim));
                } else if ("OBJECT_DETECTION".equals(cam.type)) {
                    objectCameras.put(cam.name, isEx ? new ExperimentalObjectDetectionCamera(cam.name) : new ObjectDetectionCamera(cam.name));
                }
                System.out.println("Vision: Loaded " + cam.name + " (" + cam.streamType + ")");
            }
        } catch (IOException e) { e.printStackTrace(); }
    }

    /**
     * Internal method to process vision data.
     * Calls the Subsystem's addVisionMeasurement, which safely handles Mock vs Real modes.
     */
    private void updatePoseEstimation() {
        Pose2d currentPose = swerve.getPose();
        
        // Sim Logic (Only runs if we have a valid SwerveDrive object for sim)
        if (RobotBase.isSimulation() && visionSim != null && swerve.getSwerveDrive() != null) {
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

    public List<Rotation2d> getTargetAngles(String cameraName) {
        ObjectDetectionCamera cam = objectCameras.get(cameraName);
        List<Rotation2d> angles = new ArrayList<>();
        if (cam != null) {
            var result = cam.getLatestResult();
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.getTargets()) angles.add(Rotation2d.fromDegrees(target.getYaw()));
            }
        }
        return angles;
    }

    public Optional<PhotonTrackedTarget> getBestTarget(String cameraName) {
        ObjectDetectionCamera cam = objectCameras.get(cameraName);
        return cam != null ? cam.getBestTarget() : Optional.empty();
    }
}