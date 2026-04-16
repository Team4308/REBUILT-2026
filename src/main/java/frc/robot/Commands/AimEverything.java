package frc.robot.Commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.Util.FieldConstants;

public class AimEverything extends Command {
    private final IndexerSubsystem m_IndexerSubsystem;
    private final HoodSubsystem m_HoodSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;
    private final TurretSubsystem m_TurretSubsystem;
    private final SwerveSubsystem drivebase;

    private final Supplier<Pose2d> m_swervePose;
    private final Supplier<ChassisSpeeds> m_swerveVelocity;

    // Offset of the shooter/launcher from the robot center (match your
    // TurretConstants)
    private static final Translation2d shooterOffset = new Translation2d(-0.16, 0.0);

    private static final InterpolatingTreeMap<Double, Double> hoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap passingflywheelSpeedMap = new InterpolatingDoubleTreeMap();

    private String side = "Left";
    private double powerOffset = 0;

    static {
        // Key = distance (meters), Value = hood angle (degrees from horizontal)
        hoodAngleMap.put(1.3, 8.5);
        hoodAngleMap.put(1.6, 12.5);
        hoodAngleMap.put(1.9, 13.5);
        hoodAngleMap.put(2.3, 14.5);
        hoodAngleMap.put(2.6, 15.5);
        hoodAngleMap.put(2.9, 16.5);
        hoodAngleMap.put(3.3, 17.);
        hoodAngleMap.put(3.6, 17.5);
        hoodAngleMap.put(3.9, 18.0);
        hoodAngleMap.put(4.3, 18.5);
        hoodAngleMap.put(4.6, 19.0);
        hoodAngleMap.put(4.9, 19.0);
        hoodAngleMap.put(5.2, 19.5);
        hoodAngleMap.put(4.5, 19.5);
        hoodAngleMap.put(5.5, 20.0);
        hoodAngleMap.put(6.0, 20.0);
        hoodAngleMap.put(7.0, 21.0);
        hoodAngleMap.put(8.0, 22.0);
        hoodAngleMap.put(9.0, 22.5);
        hoodAngleMap.put(10.0, 23.0);
        hoodAngleMap.put(11.0, 23.5);
        hoodAngleMap.put(12.0, 24.0);
        hoodAngleMap.put(13.0, 24.5);
        hoodAngleMap.put(14.0, 25.0);
        hoodAngleMap.put(15.0, 25.5);
        hoodAngleMap.put(16.0, 26.0);

        passingflywheelSpeedMap.put(1., 1000.);
        passingflywheelSpeedMap.put(2., 1200.);
        passingflywheelSpeedMap.put(3., 1400.);
        passingflywheelSpeedMap.put(4., 1600.);
        passingflywheelSpeedMap.put(5., 1800.);
        passingflywheelSpeedMap.put(6., 2000.);
        passingflywheelSpeedMap.put(7., 2200.);
        passingflywheelSpeedMap.put(8., 2400.);
        passingflywheelSpeedMap.put(9., 2600.);
        passingflywheelSpeedMap.put(10., 2800.);
        passingflywheelSpeedMap.put(11., 3000.);
        passingflywheelSpeedMap.put(12., 3200.);
        passingflywheelSpeedMap.put(13., 3400.);
        passingflywheelSpeedMap.put(14., 3600.);
        passingflywheelSpeedMap.put(15., 3800.);
        passingflywheelSpeedMap.put(16., 4000.);

        // Key = distance (meters), Value = shooter speed (RPM)
        flywheelSpeedMap.put(1.3, 1700.0);
        flywheelSpeedMap.put(1.6, 1750.0);
        flywheelSpeedMap.put(1.9, 1780.0);
        flywheelSpeedMap.put(2.3, 1830.0);
        flywheelSpeedMap.put(2.6, 1890.0);
        flywheelSpeedMap.put(2.9, 1980.0);
        flywheelSpeedMap.put(3.3, 2080.0);
        flywheelSpeedMap.put(3.6, 2160.0);
        flywheelSpeedMap.put(3.9, 2180.0);
        flywheelSpeedMap.put(4.3, 2240.0);
        flywheelSpeedMap.put(4.6, 2305.0);
        flywheelSpeedMap.put(4.9, 2380.);
        flywheelSpeedMap.put(5.2, 2420.);
        flywheelSpeedMap.put(5.5, 2480.);

        timeOfFlightMap.put(1.3, 0.967);
        timeOfFlightMap.put(1.6, 1.017);
        timeOfFlightMap.put(1.9, 1.017);
        timeOfFlightMap.put(2.2, 1.033);
        timeOfFlightMap.put(2.5, 1.117);
        timeOfFlightMap.put(2.8, 1.233);
        timeOfFlightMap.put(3.1, 1.317);
        timeOfFlightMap.put(3.4, 1.35);
        timeOfFlightMap.put(3.7, 1.35);
        timeOfFlightMap.put(4.0, 1.35);
        timeOfFlightMap.put(4.3, 1.35);
        timeOfFlightMap.put(4.6, 1.467);
        timeOfFlightMap.put(4.9, 1.55);
    }

    private Translation2d leftTarget = new Translation2d(2, 2);
    private Translation2d rightTarget = new Translation2d(2, 6);

    public AimEverything(
            IndexerSubsystem m_IndexerSubsystem,
            HoodSubsystem m_HoodSubsystem,
            ShooterSubsystem m_ShooterSubsystem,
            TurretSubsystem m_TurretSubsystem, Supplier<Pose2d> swervePose, Supplier<ChassisSpeeds> swerveVelocity,
            SwerveSubsystem m_s) {
        this.m_IndexerSubsystem = m_IndexerSubsystem;
        this.m_HoodSubsystem = m_HoodSubsystem;
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        this.m_TurretSubsystem = m_TurretSubsystem;
        this.m_swervePose = swervePose;
        this.m_swerveVelocity = swerveVelocity;
        this.drivebase = m_s;
        addRequirements(m_HoodSubsystem, m_ShooterSubsystem, m_TurretSubsystem);
    }

    public void setSide(String side) {
        this.side = side;
    }

    public void updatePowerOffset(double offset) {
        this.powerOffset += offset;
    }

    private void aimAtPoint(Translation2d targetPoint) {
        Pose2d pose = m_swervePose.get();
        ChassisSpeeds robotRelativeVelocity = m_swerveVelocity.get();
        Rotation2d rot = pose.getRotation();

        // Transform shooter offset from robot-relative to field-relative
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;

        // Convert robot-relative velocity to field-relative
        double vxField = robotRelativeVelocity.vxMetersPerSecond * rot.getCos()
                - robotRelativeVelocity.vyMetersPerSecond * rot.getSin();
        double vyField = robotRelativeVelocity.vxMetersPerSecond * rot.getSin()
                + robotRelativeVelocity.vyMetersPerSecond * rot.getCos();

        // Get the hub target (flipped for correct alliance)
        Translation2d targetTranslation = AllianceFlipUtil.apply(targetPoint);

        // Static distance (no lookahead) — for logging
        double dx0 = targetTranslation.getX() - shooterX;
        double dy0 = targetTranslation.getY() - shooterY;
        double staticDistance = Math.hypot(dx0, dy0);
        Logger.recordOutput("Commands/AimAtPoint/StaticDistance", staticDistance);

        // --- Iterative shoot-on-the-move solve ---
        // Each iteration: use current TOF estimate → project shooter forward →
        // recompute distance → recompute TOF
        double lookaheadShooterX = shooterX;
        double lookaheadShooterY = shooterY;
        double lookaheadDistance = staticDistance;
        double timeOfFlight = timeOfFlightMap.get(staticDistance);

        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadDistance);
            lookaheadShooterX = shooterX + vxField * timeOfFlight;
            lookaheadShooterY = shooterY + vyField * timeOfFlight;
            double ldx = targetTranslation.getX() - lookaheadShooterX;
            double ldy = targetTranslation.getY() - lookaheadShooterY;
            lookaheadDistance = Math.hypot(ldx, ldy);
        }

        // Aim from the lookahead position, not the current position
        double ldx = targetTranslation.getX() - lookaheadShooterX;
        double ldy = targetTranslation.getY() - lookaheadShooterY;
        double fieldAngleDeg = Math.toDegrees(Math.atan2(ldy, ldx));
        double turretAngleDeg = (Rotation2d.fromDegrees(fieldAngleDeg).minus(rot).getDegrees() % 360 + 360) % 360;
        m_TurretSubsystem.setTarget(turretAngleDeg);

        // Hood and shooter lookup from lookahead distance
        double hoodAngle = hoodAngleMap.get(lookaheadDistance);
        double shooterRpm = flywheelSpeedMap.get(lookaheadDistance);

        if (m_IndexerSubsystem.getTargetHopperVelocity() != 0.0) {
            m_HoodSubsystem.setHoodAngle(hoodAngle);
        } else {
            m_HoodSubsystem.setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE);
        }
        m_ShooterSubsystem.setTargetSpeed(shooterRpm + powerOffset);

        Logger.recordOutput("Commands/AimAtPoint/Target/Pose", targetPoint);
        Logger.recordOutput("Commands/AimAtPoint/Robot/Rot", rot.getDegrees());
        Logger.recordOutput("Commands/AimAtPoint/Robot/X", shooterX);
        Logger.recordOutput("Commands/AimAtPoint/Robot/Y", shooterY);
        Logger.recordOutput("Commands/AimAtPoint/Robot/Vx Field", vxField);
        Logger.recordOutput("Commands/AimAtPoint/Robot/Vy Field", vyField);
        Logger.recordOutput("Commands/AimAtPoint/Lookahead/X", lookaheadShooterX);
        Logger.recordOutput("Commands/AimAtPoint/Lookahead/Y", lookaheadShooterY);
        Logger.recordOutput("Commands/AimAtPoint/Lookahead/Distance", lookaheadDistance);
        Logger.recordOutput("Commands/AimAtPoint/Lookahead/Time Of Flight", timeOfFlight);
        Logger.recordOutput("Commands/AimAtPoint/Target/Field Deg", fieldAngleDeg);
        Logger.recordOutput("Commands/AimAtPoint/Target/Turret Deg", turretAngleDeg);
        Logger.recordOutput("Commands/AimAtPoint/Target/Static Distance", staticDistance);
        Logger.recordOutput("Commands/AimAtPoint/Target/Hood Angle", hoodAngle);
        Logger.recordOutput("Commands/AimAtPoint/Target/Shooter Rpm", shooterRpm);
        Logger.recordOutput("Commands/AimAtPoint/Power Offset", powerOffset);
    }

    private void aimAtPoint2(Translation2d targetPoint) {
        Pose2d pose = m_swervePose.get();
        ChassisSpeeds robotRelativeVelocity = m_swerveVelocity.get();
        Rotation2d rot = pose.getRotation();

        // Transform shooter offset from robot-relative to field-relative
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;

        // Convert robot-relative velocity to field-relative
        double vxField = robotRelativeVelocity.vxMetersPerSecond * rot.getCos()
                - robotRelativeVelocity.vyMetersPerSecond * rot.getSin();
        double vyField = robotRelativeVelocity.vxMetersPerSecond * rot.getSin()
                + robotRelativeVelocity.vyMetersPerSecond * rot.getCos();

        // Get the hub target (flipped for correct alliance)
        Translation2d targetTranslation = AllianceFlipUtil.apply(targetPoint);

        // Static distance (no lookahead) — for logging
        double dx0 = targetTranslation.getX() - shooterX;
        double dy0 = targetTranslation.getY() - shooterY;
        double staticDistance = Math.hypot(dx0, dy0);
        Logger.recordOutput("Commands/AimAtPoint/StaticDistance", staticDistance);

        // --- Iterative shoot-on-the-move solve ---
        // Each iteration: use current TOF estimate → project shooter forward →
        // recompute distance → recompute TOF
        double lookaheadShooterX = shooterX;
        double lookaheadShooterY = shooterY;
        double lookaheadDistance = staticDistance;
        double timeOfFlight = timeOfFlightMap.get(staticDistance);

        for (int i = 0; i < 20; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadDistance);
            lookaheadShooterX = shooterX + vxField * timeOfFlight;
            lookaheadShooterY = shooterY + vyField * timeOfFlight;
            double ldx = targetTranslation.getX() - lookaheadShooterX;
            double ldy = targetTranslation.getY() - lookaheadShooterY;
            lookaheadDistance = Math.hypot(ldx, ldy);
        }

        // Aim from the lookahead position, not the current position
        double ldx = targetTranslation.getX() - lookaheadShooterX;
        double ldy = targetTranslation.getY() - lookaheadShooterY;
        double fieldAngleDeg = Math.toDegrees(Math.atan2(ldy, ldx));
        double turretAngleDeg = (Rotation2d.fromDegrees(fieldAngleDeg).minus(rot).getDegrees() % 360 + 360) % 360;
        m_TurretSubsystem.setTarget(turretAngleDeg);

        // Hood and shooter lookup from lookahead distance
        double hoodAngle = 42.5;
        double shooterRpm = passingflywheelSpeedMap.get(lookaheadDistance);

        if (m_IndexerSubsystem.getTargetHopperVelocity() != 0.0) {
            m_HoodSubsystem.setHoodAngle(hoodAngle);
        } else {
            m_HoodSubsystem.setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE);
        }
        m_ShooterSubsystem.setTargetSpeed(shooterRpm + powerOffset);

        Logger.recordOutput("Commands/AimAtPoint/Target/Pose", targetPoint);
        Logger.recordOutput("Commands/AimAtPoint/Robot/Rot", rot.getDegrees());
        Logger.recordOutput("Commands/AimAtPoint/Robot/X", shooterX);
        Logger.recordOutput("Commands/AimAtPoint/Robot/Y", shooterY);
        Logger.recordOutput("Commands/AimAtPoint/Robot/Vx Field", vxField);
        Logger.recordOutput("Commands/AimAtPoint/Robot/Vy Field", vyField);
        Logger.recordOutput("Commands/AimAtPoint/Lookahead/X", lookaheadShooterX);
        Logger.recordOutput("Commands/AimAtPoint/Lookahead/Y", lookaheadShooterY);
        Logger.recordOutput("Commands/AimAtPoint/Lookahead/Distance", lookaheadDistance);
        Logger.recordOutput("Commands/AimAtPoint/Lookahead/Time Of Flight", timeOfFlight);
        Logger.recordOutput("Commands/AimAtPoint/Target/Field Deg", fieldAngleDeg);
        Logger.recordOutput("Commands/AimAtPoint/Target/Turret Deg", turretAngleDeg);
        Logger.recordOutput("Commands/AimAtPoint/Target/Static Distance", staticDistance);
        Logger.recordOutput("Commands/AimAtPoint/Target/Hood Angle", hoodAngle);
        Logger.recordOutput("Commands/AimAtPoint/Target/Shooter Rpm", shooterRpm);
        Logger.recordOutput("Commands/AimAtPoint/Power Offset", powerOffset);
    }

    @Override
    public void execute() {
        if (drivebase.getFieldLocation().equals("AllianceZone")) {
            aimAtPoint(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        } else {
            if (side.equals("Left")) {
                aimAtPoint2(leftTarget);
            } else if (side.equals("Right")) {
                aimAtPoint2(rightTarget);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_TurretSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}