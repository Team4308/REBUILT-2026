package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class SetIntakeAngle extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final double targetAngle;

    public SetIntakeAngle(IntakeSubsystem m_IntakeSubsystem, double targetAngle) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        this.targetAngle = targetAngle;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_IntakeSubsystem.setIntakeAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
