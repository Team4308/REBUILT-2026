package frc.robot.Commands.Trajectories;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HoodSubsystem;

public class AutoAimHood extends Command{
    private final HoodSubsystem m_HoodSubsystem;
    private final Supplier<Double> control;

    public AutoAimHood(HoodSubsystem hoodSubsystem, Supplier<Double> control) {
        this.m_HoodSubsystem = hoodSubsystem;
        this.control = control;
        addRequirements(m_HoodSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_HoodSubsystem.setHoodAngle(control.get());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
