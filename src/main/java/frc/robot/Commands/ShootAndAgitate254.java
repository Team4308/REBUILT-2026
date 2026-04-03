package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.Intake.MoveIntakeTimed;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.IntakeSubsystem.HopperStates;

public class ShootAndAgitate254 extends ParallelCommandGroup {
    public ShootAndAgitate254(IntakeSubsystem m_IntakeSubsystem, IndexerSubsystem m_IndexerSubsystem,
            TurretSubsystem m_TurretSubsystem, HoodSubsystem m_HoodSubsystem,
            Supplier<HopperStates> hopperState) {
        double time;
        if (hopperState.get() == HopperStates.FULL) {
            time = 5;
        } else if (hopperState.get() == HopperStates.HALF) {
            time = 3;
        } else {
            time = 1.5;
        }
        addCommands(
                new ShootCommand(m_IndexerSubsystem, m_TurretSubsystem, m_HoodSubsystem),
                new MoveIntakeTimed(m_IntakeSubsystem, Constants.Intake.RETRACTED_ANGLE_DEG, time));
    }

    public ShootAndAgitate254(IntakeSubsystem m_IntakeSubsystem, IndexerSubsystem m_IndexerSubsystem,
            TurretSubsystem m_TurretSubsystem, HoodSubsystem m_HoodSubsystem) {
        this(m_IntakeSubsystem, m_IndexerSubsystem, m_TurretSubsystem, m_HoodSubsystem, () -> HopperStates.FULL);
    }
}