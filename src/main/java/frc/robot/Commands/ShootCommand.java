package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class ShootCommand extends Command {
    private final IndexerSubsystem m_IndexerSubsystem;
    private final TurretSubsystem m_TurretSubsystem;
    private final HoodSubsystem m_HoodSubsystem;
    private boolean reachedTarget = true;

    public ShootCommand(IndexerSubsystem m_IndexerSubsystem, TurretSubsystem m_TurretSubsystem,
            HoodSubsystem m_HoodSubsystem) {
        this.m_IndexerSubsystem = m_IndexerSubsystem;
        this.m_TurretSubsystem = m_TurretSubsystem;
        this.m_HoodSubsystem = m_HoodSubsystem;
        addRequirements(m_IndexerSubsystem);
    }

    @Override
    public void initialize() {
        reachedTarget = false;
    }

    @Override
    public void execute() {
        double hopperVelocity = Constants.Indexer.DEFAULT_HOPPER_VELOCITY;
        double ballTunnelVelocity = Constants.Indexer.DEFAULT_INDEXER_VELOCITY;
        if (!reachedTarget) {
            hopperVelocity = 0;
        }

        if (!m_TurretSubsystem.isAtTarget() || !m_HoodSubsystem.isAtPosition()) {
            ballTunnelVelocity = 0;
            hopperVelocity = 0;
        }

        m_IndexerSubsystem.setIndexerVelocity(hopperVelocity, ballTunnelVelocity);

        if (m_IndexerSubsystem.getBallTunnelVelocity() >= 0.8 * Constants.Indexer.DEFAULT_INDEXER_VELOCITY) {
            reachedTarget = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_IndexerSubsystem.setIndexerVelocity(0, 0);
        m_IndexerSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
};