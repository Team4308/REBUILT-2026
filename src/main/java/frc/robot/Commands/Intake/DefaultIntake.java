package frc.robot.Commands.Intake;

import java.util.function.Supplier;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;

public class DefaultIntake extends Command {
    private final IntakeSubsystem m_subsystem;
    private final Supplier<Double> controlPercent;
    private final Supplier<Double> rollerSpeed;

    public DefaultIntake(IntakeSubsystem subsystem, Supplier<Double> controlPercent,
            Supplier<Double> rollerSpeed) {
        m_subsystem = subsystem;
        this.controlPercent = controlPercent;
        this.rollerSpeed = rollerSpeed;
        addRequirements(subsystem);
    }

    public DefaultIntake(IntakeSubsystem subsystem, Supplier<Double> controlPercent) {
        this(subsystem, controlPercent, () -> Constants.Intake.ROLLER_INTAKE_RPM);
    }

    @Override
    public void initialize() {
        m_subsystem.stopMotors();
    }

    @Override
    public void execute() {
        double control = this.controlPercent.get();
        m_subsystem.setIntakeAngle(DoubleUtils.mapRange(control, 0, 1, Constants.Intake.INTAKE_ANGLE_DEG,
                Constants.Intake.RETRACTED_ANGLE_DEG - 15));

        m_subsystem.setRollerSpeed(() -> this.rollerSpeed.get());
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