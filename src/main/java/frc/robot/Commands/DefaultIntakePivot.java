package frc.robot.Commands;

import java.util.function.Supplier;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;

public class DefaultIntakePivot extends Command {
    private final IntakeSubsystem m_subsystem;
    private final Supplier<Double> controlPercent;

    public DefaultIntakePivot(IntakeSubsystem subsystem, Supplier<Double> controlPercent) {
        m_subsystem = subsystem;
        this.controlPercent = controlPercent;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.stopMotors();
    }

    @Override
    public void execute() {
        double control = this.controlPercent.get();
        m_subsystem.setIntakeAngle(DoubleUtils.mapRange(control, 0, 1, Constants.Intake.INTAKE_ANGLE_DEG,
                Constants.Intake.RETRACTED_ANGLE_DEG));
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setIntakeAngle(Constants.Intake.RETRACTED_ANGLE_DEG);
        m_subsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
};