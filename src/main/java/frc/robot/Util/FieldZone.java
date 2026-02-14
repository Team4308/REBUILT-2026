package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldZone {
    private final double minX, maxX;
    private final double minY, maxY;

    public FieldZone(double minX, double maxX, double minY, double maxY) {
        this.minX = minX;
        this.maxX = maxX;
        this.minY = minY;
        this.maxY = maxY;
    }

    public boolean contains(Pose2d pose) {
        Translation2d t = pose.getTranslation();
        return t.getX() >= minX && t.getX() <= maxX &&
                t.getY() >= minY && t.getY() <= maxY;
    }
}