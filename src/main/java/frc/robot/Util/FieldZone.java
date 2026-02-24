package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class FieldZone {

    private final double minX;
    private final double maxX;
    private final double minY;
    private final double maxY;

    public FieldZone(double minX, double maxX, double minY, double maxY) {
        this.minX = Math.min(minX, maxX);
        this.maxX = Math.max(minX, maxX);
        this.minY = Math.min(minY, maxY);
        this.maxY = Math.max(minY, maxY);
    }

    /**
     * Returns true if ANY part of the robot (rotation-aware) overlaps this zone.
     */
    public boolean contains(Pose2d pose) {

        double halfWidth = Constants.BOT_WIDTH / 2.0;

        // Get robot corners
        Translation2d[] robotCorners = getRobotCorners(pose, halfWidth);

        // Get zone corners (axis-aligned)
        Translation2d[] zoneCorners = new Translation2d[] {
                new Translation2d(minX, minY),
                new Translation2d(maxX, minY),
                new Translation2d(maxX, maxY),
                new Translation2d(minX, maxY)
        };

        return satIntersects(robotCorners, zoneCorners);
    }

    private Translation2d[] getRobotCorners(Pose2d pose, double halfWidth) {

        Translation2d center = pose.getTranslation();
        Rotation2d rot = pose.getRotation();

        Translation2d[] localCorners = new Translation2d[] {
                new Translation2d(halfWidth, halfWidth),
                new Translation2d(-halfWidth, halfWidth),
                new Translation2d(-halfWidth, -halfWidth),
                new Translation2d(halfWidth, -halfWidth)
        };

        Translation2d[] worldCorners = new Translation2d[4];

        for (int i = 0; i < 4; i++) {
            worldCorners[i] = center.plus(localCorners[i].rotateBy(rot));
        }

        return worldCorners;
    }

    private boolean satIntersects(Translation2d[] a, Translation2d[] b) {
        return !hasSeparatingAxis(a, b) && !hasSeparatingAxis(b, a);
    }

    private boolean hasSeparatingAxis(Translation2d[] poly1, Translation2d[] poly2) {

        for (int i = 0; i < poly1.length; i++) {

            Translation2d p1 = poly1[i];
            Translation2d p2 = poly1[(i + 1) % poly1.length];

            // Edge normal (axis)
            Translation2d edge = p2.minus(p1);
            Translation2d axis = new Translation2d(-edge.getY(), edge.getX());

            double[] proj1 = projectPolygon(axis, poly1);
            double[] proj2 = projectPolygon(axis, poly2);

            if (proj1[1] < proj2[0] || proj2[1] < proj1[0]) {
                return true;
            }
        }

        return false;
    }

    private double[] projectPolygon(Translation2d axis, Translation2d[] poly) {

        double min = dot(poly[0], axis);
        double max = min;

        for (int i = 1; i < poly.length; i++) {
            double projection = dot(poly[i], axis);
            min = Math.min(min, projection);
            max = Math.max(max, projection);
        }

        return new double[] { min, max };
    }

    private double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }
}
