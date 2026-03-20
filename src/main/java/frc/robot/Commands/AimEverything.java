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
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.Util.FieldConstants;

public class AimEverything extends Command {
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
    private static final Translation2d shooterOffset = new Translation2d(/* x= */ 0.13, /* y= */ 0.0);

    private static final InterpolatingTreeMap<Double, Double> hoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        // Key = distance (meters), Value = hood angle (degrees from horizontal)
        hoodAngleMap.put(1.3, 8.5);
        hoodAngleMap.put(1.6, 13.2);
        hoodAngleMap.put(1.9, 15.8);
        hoodAngleMap.put(2.3, 17.);
        hoodAngleMap.put(2.6, 17.);
        hoodAngleMap.put(2.9, 17.);

        hoodAngleMap.put(3.3, 18.);
        /*
         * hoodAngleMap.put(3.6, 37.0);
         * hoodAngleMap.put(3.9, 35.0);
         * hoodAngleMap.put(4.3, 33.0);
         * hoodAngleMap.put(4.6, 31.0);
         */

        // Key = distance (meters), Value = shooter speed (RPM)
        flywheelSpeedMap.put(1.3, 1750.0);
        flywheelSpeedMap.put(1.6, 1820.0);
        flywheelSpeedMap.put(1.9, 1850.0);
        flywheelSpeedMap.put(2.3, 1950.0);
        flywheelSpeedMap.put(2.6, 2050.0);
        flywheelSpeedMap.put(2.9, 2150.0);
        flywheelSpeedMap.put(3.3, 2250.0);
        /*
         * flywheelSpeedMap.put(3.6, 3900.0);
         * flywheelSpeedMap.put(3.9, 4100.0);
         * flywheelSpeedMap.put(4.3, 4300.0);
         * flywheelSpeedMap.put(4.6, 4500.0);
         */

        timeOfFlightMap.put(1.3, 0.76);
        timeOfFlightMap.put(1.6, 0.76);
        timeOfFlightMap.put(2.2, 0.96);
        timeOfFlightMap.put(2.5, 0.75);
        timeOfFlightMap.put(2.7, 0.9);
        timeOfFlightMap.put(3., 0.9);
        timeOfFlightMap.put(3.3, 0.9);

    }

    private Translation3d leftTarget = new Translation3d(2, 2, 0);
    private Translation3d rightTarget = new Translation3d(2, 6, 0);

    public AimEverything(
            HoodSubsystem m_HoodSubsystem,
            ShooterSubsystem m_ShooterSubsystem,
            TurretSubsystem m_TurretSubsystem, Supplier<Pose2d> swervePose, Supplier<ChassisSpeeds> swerveVelocity,
            SwerveSubsystem m_s, BooleanSupplier joyRB, BooleanSupplier joyLB) {
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
        double turretAngleDeg = ((Rotation2d.fromDegrees(fieldAngleDeg).minus(rot).getDegrees()) % 360 + 540) % 360;
        m_TurretSubsystem.setTarget(turretAngleDeg);

        // Hood and shooter lookup from lookahead distance
        double hoodAngle = hoodAngleMap.get(lookaheadDistance);
        double shooterRpm = flywheelSpeedMap.get(lookaheadDistance);

        m_HoodSubsystem.setHoodAngle(hoodAngle);
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
            m_HoodSubsystem.setHoodAngle(42.5);
            m_ShooterSubsystem.setTargetSpeed(4000);
            if (joyLB.getAsBoolean()) {
                aimAtPose(leftTarget);
            } else if (joyRB.getAsBoolean()) {
                aimAtPose(rightTarget);
            }
        } else {
            m_HoodSubsystem.setHoodAngle(42.5);
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
        m_TurretSubsystem.setTarget(turretAngleDeg - 180);

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