package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import java.util.function.Supplier;

public class HoodCommand extends Command {
    private final HoodSubsystem m_hood; 
    private final Supplier<Double> m_angleSupplier;

    public HoodCommand(HoodSubsystem hood, Supplier<Double> angleSupplier) {
        this.m_hood = hood;
        this.m_angleSupplier = angleSupplier;
        addRequirements(m_hood);
    }

    @Override
    public void initialize() {
        m_hood.setState("Moving");
    }

    @Override
    public void execute() {
        m_hood.setHoodAngle(m_angleSupplier.get());
    }

    @Override
    public boolean isFinished() {
        return m_hood.isAtPosition();
    }

    @Override
    public void end(boolean interrupted) {
        m_hood.setState("Idle");
    }
}