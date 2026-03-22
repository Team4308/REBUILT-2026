package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;

public class BackupShootGroup extends SequentialCommandGroup {
    public BackupShootGroup(HoodSubsystem m_HoodSubsystem, TurretSubsystem m_TurretSubsystem,
            ShooterSubsystem m_ShooterSubsystem, SwerveSubsystem m_drivebase) {
        addCommands(
                new BackupShoot(m_HoodSubsystem, m_TurretSubsystem, m_ShooterSubsystem));
    }
}
