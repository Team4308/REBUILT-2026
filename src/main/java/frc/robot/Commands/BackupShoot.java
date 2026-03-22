package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;

/*
 * uses the best value (1.6 or something idk ill check)
 * then drives to the nearest pose thats 1.6 m away from the hub and just shoots like that
 */

public class BackupShoot extends Command {
    private final HoodSubsystem m_HoodSubsystem;
    private final TurretSubsystem m_TurretSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;

    public BackupShoot(HoodSubsystem m_HoodSubsystem, TurretSubsystem m_TurretSubsystem,
            ShooterSubsystem m_ShooterSubsystem) {
        this.m_HoodSubsystem = m_HoodSubsystem;
        this.m_TurretSubsystem = m_TurretSubsystem;
        this.m_ShooterSubsystem = m_ShooterSubsystem;

        addRequirements(m_HoodSubsystem, m_TurretSubsystem, m_ShooterSubsystem);
    }

    @Override
    public void initialize() {
        m_HoodSubsystem.setHoodAngle(12.5);
        m_ShooterSubsystem.setTargetSpeed(2800);
        m_TurretSubsystem.setTarget(360);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
