package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.Subsystems.swerve.SwerveSubsystem;
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
    private final BooleanSupplier joyRB;
    private final BooleanSupplier joyLB;

    // Offset of the shooter/launcher from the robot center (match your
    // TurretConstants)
    private static final Translation2d shooterOffset = new Translation2d(-0.16, 0.0);

    private static final InterpolatingTreeMap<Double, Double> hoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

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

        // Key = distance (meters), Value = shooter speed (RPM)
        flywheelSpeedMap.put(1.3, 1700.0);
        flywheelSpeedMap.put(1.6, 1750.0);
        flywheelSpeedMap.put(1.9, 1780.0);
        flywheelSpeedMap.put(2.3, 1830.0);
        flywheelSpeedMap.put(2.6, 1890.0);
        flywheelSpeedMap.put(2.9, 1980.0);
        flywheelSpeedMap.put(3.3, 2080.0);
        flywheelSpeedMap.put(3.6, 2160.0);
        flywheelSpeedMap.put(3.9, 2200.0);
        flywheelSpeedMap.put(4.3, 2300.0);
        flywheelSpeedMap.put(4.6, 2330.0);
        flywheelSpeedMap.put(4.9, 23800.);
        flywheelSpeedMap.put(5.2, 2420.);
        flywheelSpeedMap.put(5.5, 2480.);

        timeOfFlightMap.put(1.3, 0.8);
        timeOfFlightMap.put(1.6, 0.9);
        timeOfFlightMap.put(2.2, 1.);
        timeOfFlightMap.put(2.5, 1.);
        timeOfFlightMap.put(2.7, 1.1);
        timeOfFlightMap.put(3., 1.1);
        timeOfFlightMap.put(3.3, 1.1);
        timeOfFlightMap.put(3.6, 1.2);
        timeOfFlightMap.put(3.9, 1.3);
        timeOfFlightMap.put(4.2, 1.4);
        timeOfFlightMap.put(4.5, 1.6);
        timeOfFlightMap.put(4.8, 1.7);
        timeOfFlightMap.put(5.2, 2.);
        timeOfFlightMap.put(5.5, 2.2);
    }

    private Translation3d leftTarget = new Translation3d(2, 2, 0);
    private Translation3d rightTarget = new Translation3d(2, 6, 0);

    public AimEverything(
            IndexerSubsystem m_IndexerSubsystem,
            HoodSubsystem m_HoodSubsystem,
            ShooterSubsystem m_ShooterSubsystem,
            TurretSubsystem m_TurretSubsystem, Supplier<Pose2d> swervePose, Supplier<ChassisSpeeds> swerveVelocity,
            SwerveSubsystem m_s, BooleanSupplier joyRB, BooleanSupplier joyLB) {
        this.m_IndexerSubsystem = m_IndexerSubsystem;
        this.m_HoodSubsystem = m_HoodSubsystem;
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        this.m_TurretSubsystem = m_TurretSubsystem;
        this.m_swervePose = swervePose;
        this.m_swerveVelocity = swerveVelocity;
        this.drivebase = m_s;
        this.joyLB = joyLB;
        this.joyRB = joyRB;
        addRequirements(m_HoodSubsystem, m_ShooterSubsystem, m_TurretSubsystem);
    }

    @Override
    public void initialize() {
        leftTarget = AllianceFlipUtil.apply(leftTarget);
        rightTarget = AllianceFlipUtil.apply(rightTarget);
    }

    private void hubAim() {
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
        Translation2d hubTranslation = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        // Static distance (no lookahead) — for logging
        double dx0 = hubTranslation.getX() - shooterX;
        double dy0 = hubTranslation.getY() - shooterY;
        double staticDistance = Math.hypot(dx0, dy0);

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
            double ldx = hubTranslation.getX() - lookaheadShooterX;
            double ldy = hubTranslation.getY() - lookaheadShooterY;
            lookaheadDistance = Math.hypot(ldx, ldy);
        }

        // Aim from the lookahead position, not the current position
        double ldx = hubTranslation.getX() - lookaheadShooterX;
        double ldy = hubTranslation.getY() - lookaheadShooterY;
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
        m_ShooterSubsystem.setTargetSpeed(shooterRpm);

        Logger.recordOutput("Commands/AimAtHub/Robot/Rot", rot.getDegrees());
        Logger.recordOutput("Commands/AimAtHub/Robot/X", shooterX);
        Logger.recordOutput("Commands/AimAtHub/Robot/Y", shooterY);
        Logger.recordOutput("Commands/AimAtHub/Robot/VxField", vxField);
        Logger.recordOutput("Commands/AimAtHub/Robot/VyField", vyField);
        Logger.recordOutput("Commands/AimAtHub/Lookahead/X", lookaheadShooterX);
        Logger.recordOutput("Commands/AimAtHub/Lookahead/Y", lookaheadShooterY);
        Logger.recordOutput("Commands/AimAtHub/Lookahead/Distance", lookaheadDistance);
        Logger.recordOutput("Commands/AimAtHub/Lookahead/TimeOfFlight", timeOfFlight);
        Logger.recordOutput("Commands/AimAtHub/Target/FieldDeg", fieldAngleDeg);
        Logger.recordOutput("Commands/AimAtHub/Target/TurretDeg", turretAngleDeg);
        Logger.recordOutput("Commands/AimAtHub/Target/StaticDistance", staticDistance);
        Logger.recordOutput("Commands/AimAtHub/Target/HoodAngle", hoodAngle);
        Logger.recordOutput("Commands/AimAtHub/Target/ShooterRpm", shooterRpm);
    }

    @Override
    public void execute() {
        if (drivebase.getFieldLocation().equals("AllianceZone")) {
            hubAim();
        } else if (drivebase.getFieldLocation().equals("NeutralZone")) {
            if (m_IndexerSubsystem.getTargetHopperVelocity() != 0.0) {
                m_HoodSubsystem.setHoodAngle(42.5);
            } else {
                m_HoodSubsystem.setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE);
            }
            m_ShooterSubsystem.setTargetSpeed(4000);
            if (joyLB.getAsBoolean()) {
                aimAtPose(leftTarget);
            } else if (joyRB.getAsBoolean()) {
                aimAtPose(rightTarget);
            }
        } else {
            if (m_IndexerSubsystem.getTargetHopperVelocity() != 0.0) {
                m_HoodSubsystem.setHoodAngle(42.5);
            } else {
                m_HoodSubsystem.setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE);
            }
            m_ShooterSubsystem.setTargetSpeed(6000);
            if (joyLB.getAsBoolean()) {
                aimAtPose(leftTarget);
            } else if (joyRB.getAsBoolean()) {
                aimAtPose(rightTarget);
            }
        }
    }

    private void aimAtPose(Translation3d targetPose) {
        Pose2d botPose = drivebase.getPose();
        Pose2d pose = new Pose2d(botPose.getX(), botPose.getY(),
                botPose.getRotation());
        Rotation2d rot = pose.getRotation();
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;
        double dx = targetPose.getX() - shooterX;
        double dy = targetPose.getY() - shooterY;
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngleDeg = ((Rotation2d.fromDegrees(fieldAngleDeg).minus(rot).getDegrees()) % 360 + 360)
                % 360;
        m_TurretSubsystem.setTarget(turretAngleDeg);

        Logger.recordOutput("Commands/AimAtPose/Robot/Rot", rot.getDegrees());
        Logger.recordOutput("Commands/AimAtPose/Robot/X", shooterX);
        Logger.recordOutput("Commands/AimAtPose/Robot/Y", shooterY);
        Logger.recordOutput("Commands/AimAtPose/Target/FieldDeg", fieldAngleDeg);
        Logger.recordOutput("Commands/AimAtPose/Target/TurretDeg", turretAngleDeg);
        Logger.recordOutput("Commands/AimAtPose/Target/dX", dx);
        Logger.recordOutput("Commands/AimAtPose/Robot/dY", dy);
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