package frc.robot.Commands;

import java.util.function.Supplier;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IndexerSubsystem;

public class IndexerCommand extends Command {
    private final IndexerSubsystem m_subsystem;
    private final Supplier<Double> control;

    public IndexerCommand(IndexerSubsystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.stopMotors();
    }

    @Override
    public void execute() {
        double control = this.control.get();
        m_subsystem.setHopperVelocity(control);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
};