package frc.robot.Commands;

import java.util.function.Supplier;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.Subsystems.TurretSubsystem;

public class MoveTurretCommand extends Command {
    private final TurretSubsystem m_subsystem;
    private final Supplier<Double> angleDifference;

    public MoveTurretCommand(TurretSubsystem subsystem, Supplier<Double> angleDifference) {
        m_subsystem = subsystem;
        this.angleDifference = angleDifference;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        double targetAngle = m_subsystem.getAngleWrapped() + angleDifference.get();
        m_subsystem.setTarget(targetAngle);
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
        return m_subsystem.isAtTarget();
    }
}