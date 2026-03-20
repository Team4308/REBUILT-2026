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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.Util.FieldConstants;

public class AimHoodAndTurret extends Command {
    private final HoodSubsystem m_HoodSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;
    private final TurretSubsystem m_TurretSubsystem;

    private final Supplier<Pose2d> m_swervePose;

    // Offset of the shooter/launcher from the robot center (match your
    // TurretConstants)
    private static final Translation2d shooterOffset = new Translation2d(/* x= */ 0.0, /* y= */ 0.0);

    private static final InterpolatingTreeMap<Double, Double> hoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble());
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();

    static {
        // Key = distance (meters), Value = hood angle (degrees from horizontal)
        hoodAngleMap.put(1.3, 7.5);
        hoodAngleMap.put(1.6, 10.);
        hoodAngleMap.put(1.9, 54.0);
        hoodAngleMap.put(2.3, 50.0);
        hoodAngleMap.put(2.6, 46.0);
        hoodAngleMap.put(2.9, 43.0);
        hoodAngleMap.put(3.3, 40.0);
        hoodAngleMap.put(3.6, 37.0);
        hoodAngleMap.put(3.9, 35.0);
        hoodAngleMap.put(4.3, 33.0);
        hoodAngleMap.put(4.6, 31.0);

        // Key = distance (meters), Value = shooter speed (RPM)
        flywheelSpeedMap.put(1.3, 2200.0);
        flywheelSpeedMap.put(1.6, 2275.0);
        flywheelSpeedMap.put(1.9, 2900.0);
        flywheelSpeedMap.put(2.3, 3100.0);
        flywheelSpeedMap.put(2.6, 3300.0);
        flywheelSpeedMap.put(2.9, 3500.0);
        flywheelSpeedMap.put(3.3, 3700.0);
        flywheelSpeedMap.put(3.6, 3900.0);
        flywheelSpeedMap.put(3.9, 4100.0);
        flywheelSpeedMap.put(4.3, 4300.0);
        flywheelSpeedMap.put(4.6, 4500.0);
    }

    public AimHoodAndTurret(
            HoodSubsystem m_HoodSubsystem,
            ShooterSubsystem m_ShooterSubsystem,
            TurretSubsystem m_TurretSubsystem, Supplier<Pose2d> swervePose) {
        this.m_HoodSubsystem = m_HoodSubsystem;
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        this.m_TurretSubsystem = m_TurretSubsystem;
        this.m_swervePose = swervePose;
        addRequirements(m_HoodSubsystem, m_ShooterSubsystem, m_TurretSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Pose2d pose = m_swervePose.get();
        Rotation2d rot = pose.getRotation();

        // Transform shooter offset from robot-relative to field-relative
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;

        // Get the hub target (flipped for correct alliance)
        Translation2d hubTranslation = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

        // Distance from shooter to hub
        double dx = hubTranslation.getX() - shooterX;
        double dy = hubTranslation.getY() - shooterY;
        double distance = Math.hypot(dx, dy);

        // Turret angle
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngleDeg = ((Rotation2d.fromDegrees(fieldAngleDeg).minus(rot).getDegrees()) % 360 + 360) % 360;
        m_TurretSubsystem.setTarget(turretAngleDeg);

        // Hood and shooter lookup from distance
        double hoodAngle = hoodAngleMap.get(distance);
        double shooterRpm = flywheelSpeedMap.get(distance);

        m_HoodSubsystem.setHoodAngle(hoodAngle);
        m_ShooterSubsystem.setShooterSpeed(() -> shooterRpm);

        Logger.recordOutput("Commands/AimAtHub/Robot/Rot", rot.getDegrees());
        Logger.recordOutput("Commands/AimAtHub/Robot/X", shooterX);
        Logger.recordOutput("Commands/AimAtHub/Robot/Y", shooterY);
        Logger.recordOutput("Commands/AimAtHub/Target/FieldDeg", fieldAngleDeg);
        Logger.recordOutput("Commands/AimAtHub/Target/TurretDeg", turretAngleDeg);
        Logger.recordOutput("Commands/AimAtHub/Target/dX", dx);
        Logger.recordOutput("Commands/AimAtHub/Target/dY", dy);
        Logger.recordOutput("Commands/AimAtHub/Target/Distance", distance);
        Logger.recordOutput("Commands/AimAtHub/Target/HoodAngle", hoodAngle);
        Logger.recordOutput("Commands/AimAtHub/Target/ShooterRpm", shooterRpm);
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