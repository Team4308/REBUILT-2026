package frc.robot.Commands.Hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HoodSubsystem;

public class SetHoodAngle extends Command {
    private final HoodSubsystem m_HoodSubsystem;
    private final double targetAngle;

    public SetHoodAngle(HoodSubsystem m_HoodSubsystem, double targetAngle) {
        this.m_HoodSubsystem = m_HoodSubsystem;
        this.targetAngle = targetAngle;
        addRequirements(m_HoodSubsystem);
    }

    @Override
    public void initialize() {
        m_HoodSubsystem.setHoodAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
