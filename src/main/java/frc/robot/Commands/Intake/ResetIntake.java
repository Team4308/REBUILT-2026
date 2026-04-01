package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;

public class ResetIntake extends Command {
    private final IntakeSubsystem m_IntakeSubsystem;

    private final boolean isFinished = false;

    public ResetIntake(IntakeSubsystem m_IntakeSubsystem) {
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        addRequirements(m_IntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_IntakeSubsystem.setEnabled(false);
    }

    @Override
    public void execute() {
        if (m_IntakeSubsystem.getPivotSupplyCurrent() < Constants.Intake.AMP_THRESHOLD) {
            m_IntakeSubsystem.setPivotVoltage(-2.0);
        } else {
            m_IntakeSubsystem.setPivotVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setEnabled(true);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
