package frc.robot.subsystems.vision;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionCamera {
    protected final String name;
    protected final PhotonCamera photonCamera;

    public ObjectDetectionCamera(String name) {
        this.name = name;
        this.photonCamera = new PhotonCamera(name);
    }

    public PhotonPipelineResult getLatestResult() {
        var results = photonCamera.getAllUnreadResults();
        return results.isEmpty() ? new PhotonPipelineResult() : results.get(results.size() - 1);
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        var result = getLatestResult(); 
        return result.hasTargets() ? Optional.of(result.getBestTarget()) : Optional.empty();
    }
    
    public String getName() { return name; }
}