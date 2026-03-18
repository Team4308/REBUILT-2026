// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Shooting.Turret;
import frc.robot.Util.Field.AllianceFlipUtil;
import frc.robot.Util.Field.Bounds;
import frc.robot.Util.Field.GeomUtil;
import swervelib.SwerveDrive;
import frc.robot.Util.Field.FieldConstants;

public class LaunchCalculator {
  private static LaunchCalculator instance;

  private double hoodAngleOffsetDeg = 0.0;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.deltaTime));
  private final LinearFilter turretAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.deltaTime));

  private double lastHoodAngle;
  private Rotation2d lastturretAngle;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d turretAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
    Translation2d target,
    boolean passingLeft,
      boolean passing) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;
  private static final double phaseDelay;
  private static double hubOffset;
  private static double speedOffset;

  // Launching Maps
  private static final InterpolatingTreeMap<Double, Double> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Double> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingTreeMap<Double, Double> passingLeftHoodAngleMap =
    new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final InterpolatingDoubleTreeMap passingLeftFlywheelSpeedMap =
    new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingLeftTimeOfFlightMap =
    new InterpolatingDoubleTreeMap();
  private static final InterpolatingTreeMap<Double, Double> passingRightHoodAngleMap =
    new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());
  private static final InterpolatingDoubleTreeMap passingRightFlywheelSpeedMap =
    new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingRightTimeOfFlightMap =
    new InterpolatingDoubleTreeMap();

  // Presets
  public static final double hubPresetDistance = 0.96;
  public static final double towerPresetDistance = 2.5;
  public static final double trenchPresetDistance = 3.03;
  public static final double outpostPresetDistance = 4.84;
  public static final double passingPresetDistance = 7.0;
//   public static final LaunchPreset passingPreset;
//   public static final LaunchPreset hubPreset;
//   public static final LaunchPreset towerPreset;
//   public static final LaunchPreset trenchPreset;
//   public static final LaunchPreset outpostPreset;
//   public static final LaunchPreset hoodMinPreset =
//       new LaunchPreset(
//           new LoggedTunableNumber(
//               "LaunchCalculator/Presets/HoodMin/HoodAngle", Units.radiansToDegrees(Hood.minAngle)),
//           new LoggedTunableNumber("LaunchCalculator/Presets/HoodMin/FlywheelSpeed", 50));
//   public static final LaunchPreset hoodMaxPreset =
//       new LaunchPreset(
//           new LoggedTunableNumber(
//               "LaunchCalculator/Presets/HoodMax/HoodAngle", Units.radiansToDegrees(Hood.maxAngle)),
//           new LoggedTunableNumber("LaunchCalculator/Presets/HoodMax/FlywheelSpeed", 50));

//   public static final LoggedTunableNumber passingIdleSpeed =
//       new LoggedTunableNumber("LaunchCalculator/PassingIdleSpeed", 50);

//   public static record LaunchPreset(
//       LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}

  // Passing targets
  private static final double xPassTarget = Units.inchesToMeters(37);
  private static final double yPassTarget = Units.inchesToMeters(65);
  // Boxes of bad
  // Under tower
  private static final Bounds towerBound =
      new Bounds(0, Units.inchesToMeters(46), Units.inchesToMeters(129), Units.inchesToMeters(168));

  // Behind the hubs
  private static final Bounds nearHubBound =
      new Bounds(
          FieldConstants.LinesVertical.neutralZoneNear,
          FieldConstants.LinesVertical.neutralZoneNear + Units.inchesToMeters(120),
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);
  private static final Bounds farHubBound =
      new Bounds(
          FieldConstants.LinesVertical.oppAllianceZone,
          FieldConstants.fieldLength,
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);

  static {
    minDistance = 1.3;
    maxDistance = 3.3;
    passingMinDistance = 5.4;
    passingMaxDistance = 17.16;
    phaseDelay = 0.03;
    hubOffset = 0; //0.584
    speedOffset = 0;

    hoodAngleMap.put(1.3 + hubOffset, 0.1);
    hoodAngleMap.put(1.6 + hubOffset, 0.1);
    hoodAngleMap.put(1.9 + hubOffset, 0.1);
    hoodAngleMap.put(2.2 + hubOffset, 0.1);
    hoodAngleMap.put(2.5 + hubOffset, 0.5);
    hoodAngleMap.put(2.7 + hubOffset, 0.5);
    hoodAngleMap.put(3 + hubOffset, 0.6);
    hoodAngleMap.put(3.3 + hubOffset, 0.0);
    
    flywheelSpeedMap.put(1.3 + hubOffset, 50 + speedOffset);
    flywheelSpeedMap.put(1.6 + hubOffset, 55 + speedOffset);
    flywheelSpeedMap.put(1.9 + hubOffset, 55 + speedOffset);
    flywheelSpeedMap.put(2.2 + hubOffset, 58 + speedOffset);
    flywheelSpeedMap.put(2.5 + hubOffset, 55 + speedOffset);
    flywheelSpeedMap.put(2.5 + hubOffset, 55 + speedOffset);
    flywheelSpeedMap.put(3 + hubOffset, 60 + speedOffset);
    flywheelSpeedMap.put(3.3 + hubOffset, 60 + speedOffset);

    timeOfFlightMap.put(1.3 + hubOffset, 0.76);
    timeOfFlightMap.put(1.6 + hubOffset, 0.33);
    timeOfFlightMap.put(2.2 + hubOffset, 0.96);
    timeOfFlightMap.put(2.5 + hubOffset, 0.75);
    timeOfFlightMap.put(2.7 + hubOffset, 0.9);
    timeOfFlightMap.put(3 + hubOffset, 0.9);
    timeOfFlightMap.put(3.3 + hubOffset, 0.9);

    // TODO: passing
    passingHoodAngleMap.put(5.46, 38.0);
    passingHoodAngleMap.put(6.62, 38.0);
    passingHoodAngleMap.put(7.80, 38.0);
    passingHoodAngleMap.put(17.16, 38.0);

  passingLeftHoodAngleMap.put(5.46, 38.0);
  passingLeftHoodAngleMap.put(6.62, 38.0);
  passingLeftHoodAngleMap.put(7.80, 38.0);
  passingLeftHoodAngleMap.put(17.16, 38.0);

  passingRightHoodAngleMap.put(5.46, 38.0);
  passingRightHoodAngleMap.put(6.62, 38.0);
  passingRightHoodAngleMap.put(7.80, 38.0);
  passingRightHoodAngleMap.put(17.16, 38.0);

    passingFlywheelSpeedMap.put(5.46, 160.0);
    passingFlywheelSpeedMap.put(6.62, 180.0);
    passingFlywheelSpeedMap.put(7.80, 200.0);
    passingFlywheelSpeedMap.put(17.16, 360.0);

  passingLeftFlywheelSpeedMap.put(5.46, 160.0);
  passingLeftFlywheelSpeedMap.put(6.62, 180.0);
  passingLeftFlywheelSpeedMap.put(7.80, 200.0);
  passingLeftFlywheelSpeedMap.put(17.16, 360.0);

  passingRightFlywheelSpeedMap.put(5.46, 160.0);
  passingRightFlywheelSpeedMap.put(6.62, 180.0);
  passingRightFlywheelSpeedMap.put(7.80, 200.0);
  passingRightFlywheelSpeedMap.put(17.16, 360.0);

    passingTimeOfFlightMap.put(5.46, 1.27);
    passingTimeOfFlightMap.put(6.62, 1.39);
    passingTimeOfFlightMap.put(7.8, 1.49);
    passingTimeOfFlightMap.put(11.0, 1.75);
    passingTimeOfFlightMap.put(13.0, 1.76);
    passingTimeOfFlightMap.put(17.16, 2.16);

  passingLeftTimeOfFlightMap.put(5.46, 1.27);
  passingLeftTimeOfFlightMap.put(6.62, 1.39);
  passingLeftTimeOfFlightMap.put(7.8, 1.49);
  passingLeftTimeOfFlightMap.put(11.0, 1.75);
  passingLeftTimeOfFlightMap.put(13.0, 1.76);
  passingLeftTimeOfFlightMap.put(17.16, 2.16);

  passingRightTimeOfFlightMap.put(5.46, 1.27);
  passingRightTimeOfFlightMap.put(6.62, 1.39);
  passingRightTimeOfFlightMap.put(7.8, 1.49);
  passingRightTimeOfFlightMap.put(11.0, 1.75);
  passingRightTimeOfFlightMap.put(13.0, 1.76);
  passingRightTimeOfFlightMap.put(17.16, 2.16);

    // passingPreset =
    //     new LaunchPreset(
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Passing/HoodAngle",
    //             hoodAngleMap.get(passingPresetDistance).getDegrees()),
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Passing/FlywheelSpeed",
    //             flywheelSpeedMap.get(passingPresetDistance)));
    // hubPreset =
    //     new LaunchPreset(
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Hub/HoodAngle",
    //             hoodAngleMap.get(hubPresetDistance).getDegrees()),
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Hub/FlywheelSpeed",
    //             flywheelSpeedMap.get(hubPresetDistance)));
    // towerPreset =
    //     new LaunchPreset(
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Tower/HoodAngle",
    //             hoodAngleMap.get(towerPresetDistance).getDegrees()),
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Tower/FlywheelSpeed",
    //             flywheelSpeedMap.get(towerPresetDistance)));
    // trenchPreset =
    //     new LaunchPreset(
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Trench/HoodAngle",
    //             hoodAngleMap.get(trenchPresetDistance).getDegrees()),
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Trench/FlywheelSpeed",
    //             flywheelSpeedMap.get(trenchPresetDistance)));
    // outpostPreset =
    //     new LaunchPreset(
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Outpost/HoodAngle",
    //             hoodAngleMap.get(outpostPresetDistance).getDegrees()),
    //         new LoggedTunableNumber(
    //             "LaunchCalculator/Presets/Outpost/FlywheelSpeed",
    //             flywheelSpeedMap.get(outpostPresetDistance)));
  }

  public static void setHubOffset(double newValue){
    hubOffset = newValue;
  }
  public static void setSpeedOffset(double newValue){
    speedOffset = newValue;
  }

  public static double getSpeedOffset(){
    return speedOffset;
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public LaunchingParameters getParameters(SwerveDrive swerve, Double turretRad) {
    boolean passing =
        AllianceFlipUtil.applyX(swerve.getPose().getX())
            > FieldConstants.LinesVertical.hubCenter;
    passing = false; //TODO: we r not always passing

    // Recompute launch parameters each call so the target pose updates as the robot moves.

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = swerve.getPose();
    ChassisSpeeds robotRelativeVelocity = swerve.getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

  // Calculate target
  Translation2d target =
    passing
      ? getPassingTarget(swerve)
      : AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

  // Determine whether the passing target is on the left or right side of the field.
  boolean passingLeft = false;
  if (passing) {
    passingLeft = target.getY() < FieldConstants.fieldWidth / 2.0;
  }
    Pose2d launcherPosition = estimatedPose.transformBy(GeomUtil.toTransform2d(Turret.robotToTurret));
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());
    // if limelight can see tag set distance to tz 
    // if (LimelightHelpers.lookingAtHub(LimelightConstants.turretLimelight)) {
    //     Math.hypot(
    //         launcherToTargetDistance = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.turretLimelight)[2],
    //         launcherToTargetDistance = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.turretLimelight)[0]
    //     );
    // }

    // Calculate field relative launcher velocity
    var robotVelocity = swerve.getFieldVelocity();
    var robotAngle = swerve.getPose().getRotation();
    ChassisSpeeds launcherVelocity =
        DriverStation.isAutonomous()
            ? robotVelocity
            : GeomUtil.transformVelocity(
                robotVelocity, Turret.robotToTurret.getTranslation().toTranslation2d(), robotAngle);

    // Account for imparted velocity by robot (launcher) toe offset
  double timeOfFlight =
    passing
      ? (passingLeft ? passingLeftTimeOfFlightMap.get(launcherToTargetDistance)
        : passingRightTimeOfFlightMap.get(launcherToTargetDistance))
      : timeOfFlightMap.get(launcherToTargetDistance);

    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    
    // idk how to rlly fix this since they dont want to use robot pose however velocity is dependant on robot pose
  for (int i = 0; i < 20; i++) { 
    timeOfFlight =
      passing
        ? (passingLeft
          ? passingLeftTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
          : passingRightTimeOfFlightMap.get(lookaheadLauncherToTargetDistance))
        : timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double offsetX = launcherVelocity.vxMetersPerSecond * timeOfFlight;
      double offsetY = launcherVelocity.vyMetersPerSecond * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Account for launcher being off center
    Pose2d lookaheadRobotPose =
        lookaheadPose.transformBy(GeomUtil.toTransform2d(Turret.robotToTurret).inverse());
    Rotation2d turretAngle = getturretAngleWithLauncherOffset(lookaheadRobotPose, target, turretRad);

    // Calculate remaining parameters
  double hoodAngle =
    passing
      ? (passingLeft
        ? passingLeftHoodAngleMap.get(lookaheadLauncherToTargetDistance)
        : passingRightHoodAngleMap.get(lookaheadLauncherToTargetDistance))
      : hoodAngleMap.get(lookaheadLauncherToTargetDistance);
    if (lastturretAngle == null) lastturretAngle = turretAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.deltaTime);
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        turretAngleFilter.calculate(
            turretAngle.minus(lastturretAngle).getDegrees() / Constants.deltaTime);
    lastturretAngle = turretAngle;

    // Check if inside a box of bad
    var flippedPose = AllianceFlipUtil.apply(estimatedPose);
    boolean insideTowerBadBox = towerBound.contains(flippedPose.getTranslation());
    boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
    boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());
    boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub || behindFarHub);

  double flywheelVelocity =
    passing
      ? (passingLeft
        ? passingLeftFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
        : passingRightFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance))
      : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    // Constructor parameters
    latestParameters =
        new LaunchingParameters(
            outsideOfBadBoxes
                && lookaheadLauncherToTargetDistance >= (passing ? passingMinDistance : minDistance)
                && lookaheadLauncherToTargetDistance
                    <= (passing ? passingMaxDistance : maxDistance),
            turretAngle,
            driveVelocity,
            hoodAngle + hoodAngleOffsetDeg,
            hoodVelocity,
            flywheelVelocity,
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight,
      target,
            passingLeft,
            passing);

    // Log calculated values
    // Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    // Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadRobotPose);
    // Logger.recordOutput(
    //     "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);
    SmartDashboard.putNumber("Turret/Turret/LaunchCalc/distance", launcherToTargetDistance);
    SmartDashboard.putNumber("Turret/Turret/LaunchCalc/look ahead distance", lookaheadLauncherToTargetDistance);
    SmartDashboard.putNumber("Turret/Turret/LaunchCalc/angle", turretAngle.getDegrees());

    return latestParameters;
  }

  private static Rotation2d getturretAngleWithLauncherOffset(Pose2d robotPose, Translation2d target, double turretRad) {

    // if (LimelightHelpers.lookingAtHub(LimelightConstants.turretLimelight)) {

    //     double aprilTagOffset[] = LimelightHelpers.getTargetPose_CameraSpace(LimelightConstants.turretLimelight);
    //     // get angle
    //     return new Rotation2d(MathUtil.angleModulus(
    //     robotPose.getRotation().getRadians() - // + // turret is facing where the bot is facing 
    //     (turretRad - Math.toRadians(90)) - // + its angle -90d offset
    //     Math.atan2(aprilTagOffset[0], aprilTagOffset[2]) //(TURRET ANGLE) + (APRIL TAG ANGLE)
    //     // + Math.toRadians(180)
    //     ));
    // }

    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    Turret.robotToTurret.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d turretAngle =
        fieldToHubAngle.plus(hubAngle).plus(Turret.robotToTurret.getRotation().toRotation2d());
    
    return turretAngle; 
  }

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public Translation2d getPassingTarget(SwerveDrive swerve) {
    double flippedY = AllianceFlipUtil.apply(swerve.getPose()).getY();
    boolean mirror = flippedY > FieldConstants.LinesHorizontal.center;

    // Fixed passing target
    Translation2d flippedGoalTranslation =
        AllianceFlipUtil.apply(
            new Translation2d(
                xPassTarget, mirror ? FieldConstants.fieldWidth - yPassTarget : yPassTarget));

    return flippedGoalTranslation;
  }

  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   * @param forceBlue Always use the blue hub target
   * @return The target pose for the aimed robot.
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation, boolean forceBlue, double turretRad) {
    // Calculate target
    Translation2d target = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (!forceBlue) {
      target = AllianceFlipUtil.apply(target);
    }

    return new Pose2d(
        robotTranslation, getturretAngleWithLauncherOffset(GeomUtil.toPose2d(robotTranslation), target, turretRad));
  }

  /** Adjusts the hood angle offset up or down the specified amount. */
  public void incrementHoodAngleOffset(double incrementDegrees) {
    hoodAngleOffsetDeg += incrementDegrees;
  }
}