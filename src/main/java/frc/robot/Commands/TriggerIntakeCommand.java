package frc.robot.Commands;

import java.util.function.Supplier;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;

public class TriggerIntakeCommand extends Command {
    private final IntakeSubsystem m_subsystem;

    private Supplier<Double> joystickInput;

    public TriggerIntakeCommand(IntakeSubsystem subsystem, Supplier<Double> joystickInput) {
        m_subsystem = subsystem;

        this.joystickInput = joystickInput;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_subsystem.setIntakeAngle(DoubleUtils.mapRange(joystickInput.get(), 0, 1, 0, 127));
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return m_subsystem.isAtAngle();
    }
};