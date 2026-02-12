package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class ExperimentalObjectDetectionCamera extends ObjectDetectionCamera {

    private final DoubleArraySubscriber experimentalSubscriber;

    public ExperimentalObjectDetectionCamera(String name) {
        super(name);
        
        // Subscribe to Python Data: "Experimental/<Name>/Data"
        this.experimentalSubscriber = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.EXPERIMENTAL_ROOT)
            .getSubTable(name)
            .getDoubleArrayTopic("Data")
            .subscribe(new double[0], PubSubOption.keepDuplicates(true));
            
        System.out.println("Vision: [" + name + "] initialized in EXPERIMENTAL (Python) mode.");
    }

    @Override
    public PhotonPipelineResult getLatestResult() {
        double[] data = experimentalSubscriber.get();
        
        // Protocol requires Timestamp + Count
        if (data.length < 2) return new PhotonPipelineResult();

        long captureTimestamp = (long) data[0]; 
        int targetCount = (int) data[1];
        int packetSize = 5; // ClassID + Yaw + Pitch + Area + Confidence

        List<PhotonTrackedTarget> targets = new ArrayList<>();

        for (int i = 0; i < targetCount; i++) {
            int base = 2 + (i * packetSize);
            if (base + packetSize > data.length) break; 

            int classId = (int) data[base + 0];
            double yaw = data[base + 1];
            double pitch = data[base + 2];
            double area = data[base + 3];
            double confidence = data[base + 4];

            // Construct Target (No Pose for Objects)
            targets.add(new PhotonTrackedTarget(
                yaw, pitch, area, 0.0, 
                -1,                  // Fiducial ID
                classId,             // Class ID
                (float)confidence,   // Confidence
                new Transform3d(), new Transform3d(), // Empty 3D Pose
                0.0,                 // Ambiguity
                new ArrayList<>(), new ArrayList<>()  // Corners
            ));
        }

        return new PhotonPipelineResult(
            0, captureTimestamp, (long)(Timer.getFPGATimestamp() * 1e6), -1, targets
        );
    }
}