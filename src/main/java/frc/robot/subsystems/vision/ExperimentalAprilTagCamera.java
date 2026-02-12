package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class ExperimentalAprilTagCamera extends AprilTagCamera {
    private final DoubleArraySubscriber experimentalSubscriber;

    public ExperimentalAprilTagCamera(String name, Transform3d robotToCam, AprilTagFieldLayout layout, VisionSystemSim visionSim) {
        super(name, robotToCam, layout, visionSim);
        this.experimentalSubscriber = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.EXPERIMENTAL_ROOT).getSubTable(name)
            .getDoubleArrayTopic("Data").subscribe(new double[0], PubSubOption.keepDuplicates(true));
        System.out.println("Vision: [" + name + "] initialized in EXPERIMENTAL (Python) mode.");
    }

    @Override
    public PhotonPipelineResult getLatestResult() {
        double[] data = experimentalSubscriber.get();
        if (data.length < 2) return new PhotonPipelineResult();

        long captureTimestamp = (long) data[0]; 
        int targetCount = (int) data[1];
        int packetSize = 14; 
        List<PhotonTrackedTarget> targets = new ArrayList<>();

        for (int i = 0; i < targetCount; i++) {
            int base = 2 + (i * packetSize);
            if (base + packetSize > data.length) break; 
            List<TargetCorner> corners = List.of(
                new TargetCorner(data[base + 6], data[base + 7]), new TargetCorner(data[base + 8], data[base + 9]),
                new TargetCorner(data[base + 10], data[base + 11]), new TargetCorner(data[base + 12], data[base + 13])
            );
            targets.add(new PhotonTrackedTarget(data[base + 1], data[base + 2], data[base + 3], data[base + 4], (int)data[base + 0], -1, 0.0f, new Transform3d(), new Transform3d(), data[base + 5], corners, corners));
        }
        return new PhotonPipelineResult(0, captureTimestamp, (long)(Timer.getFPGATimestamp() * 1e6), -1, targets);
    }
}