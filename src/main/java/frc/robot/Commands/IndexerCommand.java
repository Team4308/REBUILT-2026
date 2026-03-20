package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;

public class IndexerCommand extends Command {
    private final IndexerSubsystem m_subsystem;
    private final Supplier<Double> rpm;

    public IndexerCommand(IndexerSubsystem subsystem, Supplier<Double> rpm) {
        m_subsystem = subsystem;
        this.rpm = rpm;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.setIndexerVelocity(rpm.get(), rpm.get());
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