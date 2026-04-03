package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Util.FieldZone;

public class FieldLayout {
        /**
         * Origin is the bottom left corner of the field image (Close right corner from
         * blue driver
         * station POV)
         */

        // Everything in Meters
        public static final double kFieldLength = Units.inchesToMeters(651.2);

        public static final double kFieldWidth = Units.inchesToMeters(317.7);
        public static final double kTapeWidth = Units.inchesToMeters(2.0);

        public static final double kCenterLineX = kFieldLength / 2.0;
        public static final double kZoneDepth = 4;
        public static final double kObstacleWidth = Units.inchesToMeters(47);
        public static final double kTrenchWidth = Units.inchesToMeters(50.59);

        public static class ShooterTargets {
                public static final double kHUB_HEIGHT = 2.6;

                public static final Translation3d kBLUE_HUB_POSE = new Translation3d(4.5, kFieldWidth / 2.0,
                                kHUB_HEIGHT);
                public static final Translation3d kRED_HUB_POSE = new Translation3d(kFieldLength - 4.5,
                                kFieldWidth / 2.0, kHUB_HEIGHT);

                public static final Translation3d kHUB_POSE = kBLUE_HUB_POSE;

                public static final Translation3d kPASS_RIGHT_POSE = new Translation3d(4.0, kFieldWidth / 2.0 + 0.5,
                                kHUB_HEIGHT);
                public static final Translation3d kPASS_LEFT_POSE = new Translation3d(4.0, kFieldWidth / 2.0 - 0.5,
                                kHUB_HEIGHT);

                public static Translation3d getAlliancePassRight() {
                        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                        return isRed ? new Translation3d(kFieldLength - kPASS_RIGHT_POSE.getX(),
                                        kPASS_RIGHT_POSE.getY(), kHUB_HEIGHT) : kPASS_RIGHT_POSE;
                }

                public static Translation3d getAlliancePassLeft() {
                        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
                        return isRed ? new Translation3d(kFieldLength - kPASS_LEFT_POSE.getX(), kPASS_LEFT_POSE.getY(),
                                        kHUB_HEIGHT) : kPASS_LEFT_POSE;
                }

                public static Translation3d getAllianceHub() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                                        ? kRED_HUB_POSE
                                        : kBLUE_HUB_POSE;
                }
        }

        public static class Zones {
                public static final FieldZone blueAllianceZone = new FieldZone(
                                0.0,
                                kZoneDepth,
                                0.0,
                                kFieldWidth);

                public static final FieldZone redAllianceZone = new FieldZone(
                                kFieldLength - kZoneDepth,
                                kFieldLength,
                                0.0,
                                kFieldWidth);

                public static final FieldZone neutralZone = new FieldZone(
                                kZoneDepth,
                                kFieldLength - kZoneDepth,
                                0.0,
                                kFieldWidth);

                public static final FieldZone redRightTrench = new FieldZone(
                                kFieldLength - kZoneDepth - kObstacleWidth,
                                kFieldLength - kZoneDepth,
                                0.0,
                                kTrenchWidth);

                public static final FieldZone redLeftTrench = new FieldZone(
                                kFieldLength - kZoneDepth - kObstacleWidth,
                                kFieldLength - kZoneDepth,
                                kFieldWidth - kTrenchWidth,
                                kFieldWidth);

                public static final FieldZone redRightBump = new FieldZone(
                                kFieldLength - kZoneDepth - kObstacleWidth,
                                kFieldLength - kZoneDepth,
                                kTrenchWidth,
                                kFieldWidth / 2.0);

                public static final FieldZone redLeftBump = new FieldZone(
                                kFieldLength - kZoneDepth - kObstacleWidth,
                                kFieldLength - kZoneDepth,
                                kFieldWidth / 2.0,
                                kFieldWidth - kTrenchWidth);

                public static final FieldZone blueRightTrench = new FieldZone(
                                kZoneDepth,
                                kZoneDepth + kObstacleWidth,
                                kFieldWidth - kTrenchWidth,
                                kFieldWidth);

                public static final FieldZone blueLeftTrench = new FieldZone(
                                kZoneDepth,
                                kZoneDepth + kObstacleWidth,
                                0.0,
                                kTrenchWidth);

                public static final FieldZone blueRightBump = new FieldZone(
                                kZoneDepth,
                                kZoneDepth + kObstacleWidth,
                                kFieldWidth / 2.0,
                                kFieldWidth - kTrenchWidth);

                public static final FieldZone blueLeftBump = new FieldZone(
                                kZoneDepth,
                                kZoneDepth + kObstacleWidth,
                                kTrenchWidth,
                                kFieldWidth / 2.0);

                public static final Pose2d blueLeftPose = new Pose2d(2.5, 6.5, new Rotation2d(0));
                public static final Pose2d blueRightPose = new Pose2d(2.5, 1.5, new Rotation2d(0));
                public static final Pose2d neutralLeftPose = new Pose2d(8.25, 6.5, new Rotation2d(0));
                public static final Pose2d neutralRightPose = new Pose2d(8.25, 1.5, new Rotation2d(0));
                public static final Pose2d redLeftPose = new Pose2d(13.5, 6.5, new Rotation2d(0));
                public static final Pose2d redRightPose = new Pose2d(13.5, 1.5, new Rotation2d(0));

                public static Pose2d getAllianceLeftPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? blueLeftPose
                                        : redRightPose;
                }

                public static Pose2d getAllianceRightPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? blueRightPose
                                        : redLeftPose;
                }

                public static Pose2d getOpponentRightPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? redRightPose
                                        : blueLeftPose;
                }

                public static Pose2d getOpponentLeftPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? redLeftPose
                                        : blueRightPose;
                }

                public static Pose2d getNeutralLeft() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? neutralLeftPose
                                        : neutralRightPose;
                }

                public static Pose2d getNeutralRight() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? neutralRightPose
                                        : neutralLeftPose;
                }

                public static Translation3d getAllianceLeftTranslation3d() {
                        return new Translation3d(getAllianceLeftPose().getX(), getAllianceLeftPose().getY(), 0.0);
                }

                public static Translation3d getAllianceRightTranslation3d() {
                        return new Translation3d(getAllianceRightPose().getX(), getAllianceRightPose().getY(), 0.0);
                }
        }

        public static class StartingPoses {
                public static final Pose2d midStartingPose = new Pose2d(3.5, kFieldWidth / 2.0, new Rotation2d(0));
                public static final Pose2d leftStartingPose = new Pose2d(3.5, 7.375, new Rotation2d(0));
                public static final Pose2d rightStartingPose = new Pose2d(3.5, 0.625, new Rotation2d(0));
                public static final Pose2d leftTrenchStartingPose = new Pose2d(4.5, 7.375, new Rotation2d(0));
                public static final Pose2d rightTrenchStartingPose = new Pose2d(4.5, 0.625, new Rotation2d(0));
                public static final Pose2d leftBumpStartingPose = new Pose2d(4.5, 5.5, new Rotation2d(0));
                public static final Pose2d rightBumpStartingPose = new Pose2d(4.5, 2.5, new Rotation2d(0));

                public static Pose2d getMidStartingPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? midStartingPose
                                        : new Pose2d(kFieldLength - midStartingPose.getX(),
                                                        midStartingPose.getY(),
                                                        midStartingPose.getRotation().unaryMinus());
                }

                public static Pose2d getLeftStartingPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? leftStartingPose
                                        : new Pose2d(kFieldLength - leftStartingPose.getX(),
                                                        leftStartingPose.getY(),
                                                        leftStartingPose.getRotation().unaryMinus());
                }

                public static Pose2d getRightStartingPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? rightStartingPose
                                        : new Pose2d(kFieldLength - rightStartingPose.getX(),
                                                        rightStartingPose.getY(),
                                                        rightStartingPose.getRotation().unaryMinus());
                }

                public static Pose2d getLeftTrenchStartingPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? leftTrenchStartingPose
                                        : new Pose2d(kFieldLength - leftTrenchStartingPose.getX(),
                                                        leftTrenchStartingPose.getY(),
                                                        leftTrenchStartingPose.getRotation().unaryMinus());
                }

                public static Pose2d getRightTrenchStartingPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? rightTrenchStartingPose
                                        : new Pose2d(kFieldLength - rightTrenchStartingPose.getX(),
                                                        rightTrenchStartingPose.getY(),
                                                        rightTrenchStartingPose.getRotation().unaryMinus());
                }

                public static Pose2d getLeftBumpStartingPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? leftBumpStartingPose
                                        : new Pose2d(kFieldLength - leftBumpStartingPose.getX(),
                                                        leftBumpStartingPose.getY(),
                                                        leftBumpStartingPose.getRotation().unaryMinus());
                }

                public static Pose2d getRightBumpStartingPose() {
                        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                        ? rightBumpStartingPose
                                        : new Pose2d(kFieldLength - rightBumpStartingPose.getX(),
                                                        rightBumpStartingPose.getY(),
                                                        rightBumpStartingPose.getRotation().unaryMinus());
                }
        }
}