package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.Intake.Agitate254;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class ShootAndAgitateJ extends ParallelCommandGroup {
    public ShootAndAgitateJ(IntakeSubsystem m_IntakeSubsystem, IndexerSubsystem m_IndexerSubsystem) {
        addCommands(
                new ShootCommand(m_IndexerSubsystem),
                new Agitate254(m_IntakeSubsystem));
    }
}