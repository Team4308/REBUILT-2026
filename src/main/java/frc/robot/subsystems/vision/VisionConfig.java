package frc.robot.subsystems.vision;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

@JsonIgnoreProperties(ignoreUnknown = true) // This allows "_comment" in JSON
public class VisionConfig {
    public List<CameraConfig> cameras;

    public static class CameraConfig {
        public String name;
        public String type;       // "APRILTAG" or "OBJECT_DETECTION"
        public String streamType; // "PHOTON" (Standard) or "PYTHON" (Experimental)
        public boolean enabled;
        public TransformData transform;
        public ResolutionData resolution;
    }

    public static class TransformData {
        public double x, y, z;          // Inches
        public double roll, pitch, yaw; // Degrees

        public Transform3d toTransform3d() {
            return new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(x),
                    Units.inchesToMeters(y),
                    Units.inchesToMeters(z)
                ),
                new Rotation3d(
                    Math.toRadians(roll),
                    Math.toRadians(pitch),
                    Math.toRadians(yaw)
                )
            );
        }
    }

    public static class ResolutionData {
        public int width;
        public int height;
        public double fov;
    }
}