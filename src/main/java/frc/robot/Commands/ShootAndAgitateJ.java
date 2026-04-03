package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.Intake.Agitate254;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class ShootAndAgitateJ extends ParallelCommandGroup {
    public ShootAndAgitateJ(IntakeSubsystem m_IntakeSubsystem, IndexerSubsystem m_IndexerSubsystem,
            TurretSubsystem m_TurretSubsystem, HoodSubsystem m_HoodSubsystem) {
        addCommands(
                new ShootCommand(m_IndexerSubsystem, m_TurretSubsystem, m_HoodSubsystem),
                new Agitate254(m_IntakeSubsystem));
    }
}