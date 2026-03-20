package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;

public class ShootCommand extends Command {
    private final IndexerSubsystem m_subsystem;

    public ShootCommand(IndexerSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setIndexerVelocity(Constants.Indexer.DEFAULT_HOPPER_VELOCITY,
                Constants.Indexer.DEFAULT_INDEXER_VELOCITY);
    }

    @Override
    public void execute() {
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