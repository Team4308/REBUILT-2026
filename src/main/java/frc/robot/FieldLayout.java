package frc.robot;

import edu.wpi.first.math.util.Units;
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

  public static final double kZoneDepth = 4; // This is not in the controlled dimensions, so we might have to
                                             // measure this.

  public static final double kObstacleWidth = Units.inchesToMeters(47);
  public static final double kTrenchWdith = Units.inchesToMeters(50.59);

  // TODO: verify all these are correct
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
      kTrenchWdith);

  public static final FieldZone redLeftTrench = new FieldZone(
      kFieldLength - kZoneDepth - kObstacleWidth,
      kFieldLength - kZoneDepth,
      kFieldWidth - kTrenchWdith,
      kFieldWidth);

  public static final FieldZone redRightBump = new FieldZone(
      kFieldLength - kZoneDepth - kObstacleWidth,
      kFieldLength - kZoneDepth,
      kTrenchWdith,
      kFieldWidth / 2.0);

  public static final FieldZone redLeftBump = new FieldZone(
      kFieldLength - kZoneDepth - kObstacleWidth,
      kFieldLength - kZoneDepth,
      kFieldWidth / 2.0,
      kFieldWidth - kTrenchWdith);

  public static final FieldZone blueRightTrench = new FieldZone(
      kZoneDepth,
      kZoneDepth + kObstacleWidth,
      kFieldWidth - kTrenchWdith,
      kFieldWidth);

  public static final FieldZone blueLeftTrench = new FieldZone(
      kZoneDepth,
      kZoneDepth + kObstacleWidth,
      0.0,
      kTrenchWdith);

  public static final FieldZone blueRightBump = new FieldZone(
      kZoneDepth,
      kZoneDepth + kObstacleWidth,
      kFieldWidth / 2.0,
      kFieldWidth - kTrenchWdith);

  public static final FieldZone blueLeftBump = new FieldZone(
      kZoneDepth,
      kZoneDepth + kObstacleWidth,
      kTrenchWdith,
      kFieldWidth / 2.0);

}
