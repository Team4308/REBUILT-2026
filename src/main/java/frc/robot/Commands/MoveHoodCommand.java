package frc.robot.Commands;

import java.util.function.Supplier;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.HoodSubsystem;

public class MoveHoodCommand extends Command {
    private final HoodSubsystem m_subsystem;
    private final Supplier<Double> angleDifference;

    private double targetAngle = 0;

    public MoveHoodCommand(HoodSubsystem subsystem, Supplier<Double> AngleDifference) {
        m_subsystem = subsystem;
        this.angleDifference = angleDifference;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double control = this.control.get();
        targetAngle += control * 2;
        targetAngle = DoubleUtils.clamp(targetAngle, Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE,
                Constants.Shooting.Hood.FORWARD_SOFT_LIMIT_ANGLE);
        m_subsystem.setHoodAngle(targetAngle);
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