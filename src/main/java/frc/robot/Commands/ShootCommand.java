package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IndexerSubsystem;

public class ShootCommand extends Command {
    private final IndexerSubsystem m_subsystem;
    private boolean reachedTarget = true;

    public ShootCommand(IndexerSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        reachedTarget = false;
    }

    @Override
    public void execute() {
        if (reachedTarget) {
            m_subsystem.setIndexerVelocity(Constants.Indexer.DEFAULT_HOPPER_VELOCITY,
                    Constants.Indexer.DEFAULT_INDEXER_VELOCITY);
        } else {
            m_subsystem.setIndexerVelocity(0, Constants.Indexer.DEFAULT_INDEXER_VELOCITY);
        }

        if (m_subsystem.getBallTunnelVelocity() >= 0.8 * m_subsystem.getTargetBallTunnelVelocity()) {
            reachedTarget = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setIndexerVelocity(0, 0);
        m_subsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
};