package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HoodSubsystem;

public class MoveHoodCommand extends Command {
    private final HoodSubsystem m_subsystem;
    private final Supplier<Double> angleDifference;

    public MoveHoodCommand(HoodSubsystem subsystem, Supplier<Double> angleDifference) {
        m_subsystem = subsystem;
        this.angleDifference = angleDifference;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        double targetAngle = m_subsystem.getHoodAngle() + angleDifference.get();
        m_subsystem.setHoodAngle(targetAngle);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.isAtPosition();
    }
};