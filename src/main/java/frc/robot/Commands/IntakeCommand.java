package frc.robot.Commands;

import java.util.function.Supplier;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_subsystem;
    private final Supplier<Double> control;

    public IntakeCommand(IntakeSubsystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.stopMotors();
    }

    @Override
    public void execute() {
        double control = this.control.get();
        m_subsystem.setRollerSpeed(() -> control * 6000);
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