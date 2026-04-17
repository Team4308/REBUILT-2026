package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class EjectBalls extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final IndexerSubsystem m_IndexerSubsystem;

    public EjectBalls(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        m_IntakeSubsystem = intakeSubsystem;
        m_IndexerSubsystem = indexerSubsystem;
        addRequirements(intakeSubsystem, indexerSubsystem);
    }

    @Override
    public void initialize() {
        m_IntakeSubsystem.setRollerSpeed(() -> -Constants.Intake.ROLLER_INTAKE_RPM);
        m_IndexerSubsystem.setIndexerVelocity(-Constants.Indexer.DEFAULT_HOPPER_VELOCITY,
                -Constants.Indexer.DEFAULT_INDEXER_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setRollerSpeed(() -> 0.);
        m_IndexerSubsystem.setIndexerVelocity(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
