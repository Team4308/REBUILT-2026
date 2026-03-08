package frc.robot.Commands;

import java.util.function.Supplier;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;

public class ToggleIntakeCommand extends Command {
    private final IntakeSubsystem m_subsystem;

    String state;

    public ToggleIntakeCommand(IntakeSubsystem subsystem) {
        m_subsystem = subsystem;

        if (Math.abs(m_subsystem.getIntakeAngle() - Constants.Intake.INTAKE_ANGLE_DEG) < 5) {
            state = "INTAKING";
        } else {
            state = "UP";
        }

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.stopMotors();
        if (state == "INTAKING") {
            m_subsystem.setIntakeAngle(Constants.Intake.RETRACTED_ANGLE_DEG);
        } else {
            m_subsystem.setIntakeAngle(Constants.Intake.INTAKE_ANGLE_DEG);
        }
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
        return m_subsystem.isAtAngle();
    }
};