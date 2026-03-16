package frc.robot.Commands.Trajectories;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterSubsystem;

public class AutoAimShooter extends Command{
    private final ShooterSubsystem m_ShooterSubsystem;
    private final Supplier<Double> control;

    public AutoAimShooter(ShooterSubsystem shooterSubsystem, Supplier<Double> control) {
        this.m_ShooterSubsystem = shooterSubsystem;
        this.control = control;
        addRequirements(m_ShooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_ShooterSubsystem.setShooterSpeed(() -> control.get());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
