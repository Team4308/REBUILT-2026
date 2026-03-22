package frc.robot.Commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.Util.FieldConstants;

/*
 * uses the best value (1.6 or something idk ill check)
 * then drives to the nearest pose thats 1.6 m away from the hub and just shoots like that
 */

public class BackupShoot extends Command {
    private final HoodSubsystem m_HoodSubsystem;
    private final TurretSubsystem m_TurretSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;
    private final SwerveSubsystem m_drivebase;

    private static final Translation2d shooterOffset = new Translation2d(/* x= */ 0.13, /* y= */ 0.0);

    public BackupShoot(HoodSubsystem m_HoodSubsystem, TurretSubsystem m_TurretSubsystem,
            ShooterSubsystem m_ShooterSubsystem, SwerveSubsystem m_driveBase) {
        this.m_HoodSubsystem = m_HoodSubsystem;
        this.m_TurretSubsystem = m_TurretSubsystem;
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        this.m_drivebase = m_driveBase;
    }

    @Override
    public void initialize() {
        m_HoodSubsystem.setHoodAngle(13.2);
        m_ShooterSubsystem.setTargetSpeed(2800);
    }

    @Override
    public void execute() {
        Pose2d pose = m_drivebase.getPose();

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
        Logger.recordOutput("Distnace", distance);

        // Turret angle
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngleDeg = ((Rotation2d.fromDegrees(fieldAngleDeg).minus(rot).getDegrees()) % 360 + 540) % 360;
        m_TurretSubsystem.setTarget(360);

        Logger.recordOutput("Commands/AimAtHub/Robot/Rot", rot.getDegrees());
        Logger.recordOutput("Commands/AimAtHub/Robot/X", shooterX);
        Logger.recordOutput("Commands/AimAtHub/Robot/Y", shooterY);
        Logger.recordOutput("Commands/AimAtHub/Target/FieldDeg", fieldAngleDeg);
        Logger.recordOutput("Commands/AimAtHub/Target/TurretDeg", turretAngleDeg);
        Logger.recordOutput("Commands/AimAtHub/Target/dX", dx);
        Logger.recordOutput("Commands/AimAtHub/Target/dY", dy);
        Logger.recordOutput("Commands/AimAtHub/Target/Distance", distance);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
