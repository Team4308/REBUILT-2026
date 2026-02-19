package frc.robot.subsystems.vision;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Wraps a standard PhotonVision camera configured for Object Detection (e.g., Note or game piece detection).
 * Provides methods to retrieve raw pipeline results and identify the best target.
 */
public class ObjectDetectionCamera {
    protected final String name;
    protected final PhotonCamera photonCamera;

    /**
     * Constructs a new ObjectDetectionCamera.
     *
     * @param name The name of the camera (must match the PhotonVision pipeline name).
     */
    public ObjectDetectionCamera(String name) {
        this.name = name;
        this.photonCamera = new PhotonCamera(name);
    }

    /**
     * Fetches the latest unread result from the camera pipeline.
     *
     * @return The latest PhotonPipelineResult, or an empty result if none exist.
     */
    public PhotonPipelineResult getLatestResult() {
        var results = photonCamera.getAllUnreadResults();
        return results.isEmpty() ? new PhotonPipelineResult() : results.get(results.size() - 1);
    }

    /**
     * Retrieves the "best" target from the latest result.
     * The definition of "best" is determined by the PhotonVision pipeline sorting (usually largest area or highest confidence).
     *
     * @return An Optional containing the best PhotonTrackedTarget, or empty if no targets are visible.
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        var result = getLatestResult(); 
        return result.hasTargets() ? Optional.of(result.getBestTarget()) : Optional.empty();
    }
    
    /**
     * Gets the name of the camera.
     * @return The camera name.
     */
    public String getName() { return name; }
}