package frc.robot.Commands.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.HoodSubsystem;

public class ResetHood extends Command {
    private final HoodSubsystem m_HoodSubsystem;

    private final boolean isFinished = false;

    public ResetHood(HoodSubsystem m_HoodSubsystem) {
        this.m_HoodSubsystem = m_HoodSubsystem;
        addRequirements(m_HoodSubsystem);
    }

    @Override
    public void initialize() {
        m_HoodSubsystem.setEnabled(false);
    }

    @Override
    public void execute() {
        if (m_HoodSubsystem.getSupplyCurrent() < Constants.Shooting.Hood.AMP_THRESHOLD) {
            m_HoodSubsystem.setHoodVoltage(-2.0);
        } else {
            m_HoodSubsystem.setHoodVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_HoodSubsystem.setEnabled(true);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
