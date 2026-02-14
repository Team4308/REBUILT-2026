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

  public static final double kDriverWallDepth = 4; // This is not in the controlled dimensions, so we might have to
                                                   // measure this.

  // TODO: verify all these are correct
  public static final FieldZone blueAllianceZone = new FieldZone(
      0.0,
      kCenterLineX,
      0.0,
      kFieldWidth);

  public static final FieldZone redAllianceZone = new FieldZone(
      kCenterLineX,
      kFieldLength,
      0.0,
      kFieldWidth);

  public static final double kNeutralHalfWidth = 0.5;

  public static final FieldZone neutralZone = new FieldZone(
      kCenterLineX - kNeutralHalfWidth,
      kCenterLineX + kNeutralHalfWidth,
      0.0,
      kFieldWidth);

  public static final FieldZone redRightTrench = new FieldZone(
      kCenterLineX,
      kFieldLength,
      0.0,
      kFieldWidth / 2.0);

  public static final FieldZone redLeftTrench = new FieldZone(
      kCenterLineX,
      kFieldLength,
      kFieldWidth / 2.0,
      kFieldWidth);

  public static final FieldZone redRightBump = new FieldZone(
      kFieldLength - kDriverWallDepth,
      kFieldLength,
      0.0,
      kFieldWidth / 2.0);

  public static final FieldZone redLeftBump = new FieldZone(
      kFieldLength - kDriverWallDepth,
      kFieldLength,
      kFieldWidth / 2.0,
      kFieldWidth);

  public static final FieldZone blueRightTrench = new FieldZone(
      0.0,
      kCenterLineX,
      0.0,
      kFieldWidth / 2.0);

  public static final FieldZone blueLeftTrench = new FieldZone(
      0.0,
      kCenterLineX,
      kFieldWidth / 2.0,
      kFieldWidth);

  public static final FieldZone blueRightBump = new FieldZone(
      0.0,
      kDriverWallDepth,
      0.0,
      kFieldWidth / 2.0);

  public static final FieldZone blueLeftBump = new FieldZone(
      0.0,
      kDriverWallDepth,
      kFieldWidth / 2.0,
      kFieldWidth);

}
