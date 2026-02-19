package frc.robot.subsystems.vision;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Configuration Data Object (POJO) mapping to the "vision-config.json" file.
 * This class defines the structure for camera settings, transforms, and resolutions.
 */
@JsonIgnoreProperties(ignoreUnknown = true) // This allows "_comment" in JSON
public class VisionConfig {
    /** List of individual camera configurations. */
    public List<CameraConfig> cameras;

    /**
     * Configuration settings for a single camera.
     */
    public static class CameraConfig {
        public String name;
        public String type;       // "APRILTAG" or "OBJECT_DETECTION"
        public String streamType; // "PHOTON" (Standard) or "PYTHON" (Experimental)
        public boolean enabled;
        public TransformData transform;
        public ResolutionData resolution;
    }

    /**
     * 3D Transform data used to define the camera's position relative to the robot center.
     */
    public static class TransformData {
        public double x, y, z;          // Inches
        public double roll, pitch, yaw; // Degrees

        /**
         * Converts the raw configuration data into a WPILib Transform3d object.
         *
         * @return Transform3d representing the camera's pose.
         */
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

    /**
     * Resolution and Field of View settings for the camera.
     */
    public static class ResolutionData {
        public int width;
        public int height;
        public double fov;
    }
}