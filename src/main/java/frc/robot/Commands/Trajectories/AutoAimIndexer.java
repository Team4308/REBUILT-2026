package frc.robot.Commands.Trajectories;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IndexerSubsystem;

public class AutoAimIndexer extends Command{
    private final IndexerSubsystem m_IndexerSubsystem;

    public AutoAimIndexer(IndexerSubsystem indexerSubsystem) {
        m_IndexerSubsystem = indexerSubsystem;
        addRequirements(m_IndexerSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean ballsReady = !m_IndexerSubsystem.getBeambreak();
        if (!ballsReady && m_IndexerSubsystem.getTargetBallTunnelVelocity() != 0 && m_IndexerSubsystem.getTargetHopperVelocity() != 0) {
            m_IndexerSubsystem.setHopperVelocity(Constants.Indexer.PASSIVE_HOPEPR_VELOCITY);
            m_IndexerSubsystem.setIndexerVelocity(Constants.Indexer.PASSIVE_INDEXER_VELOCITY);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
