package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.Intake.MoveIntakeTimed;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.HopperStates;

public class ShootAndAgitate254 extends ParallelCommandGroup {
    public ShootAndAgitate254(IntakeSubsystem m_IntakeSubsystem, IndexerSubsystem m_IndexerSubsystem,
            Supplier<HopperStates> hopperState) {
        addCommands(
                new ShootCommand(m_IndexerSubsystem),
                new MoveIntakeTimed(m_IntakeSubsystem, Constants.Intake.RETRACTED_ANGLE_DEG, 5));
    }

    public ShootAndAgitate254(IntakeSubsystem m_IntakeSubsystem, IndexerSubsystem m_IndexerSubsystem) {
        this(m_IntakeSubsystem, m_IndexerSubsystem, () -> HopperStates.FULL);
    }
}