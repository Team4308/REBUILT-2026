package frc.robot.Commands;

import java.util.function.Supplier;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.TurretSubsystem;

public class TurretCommand extends Command {
    private final TurretSubsystem m_subsystem;
    private final Supplier<Double> control;

    private double targetAngle = 0;

    public TurretCommand(TurretSubsystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double control = this.control.get();
        targetAngle += control * 2;
        targetAngle = DoubleUtils.clamp(targetAngle, Constants.Shooting.Turret.MIN_DEGREES,
                Constants.Shooting.Turret.MAX_DEGREES);
        m_subsystem.setTarget(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}