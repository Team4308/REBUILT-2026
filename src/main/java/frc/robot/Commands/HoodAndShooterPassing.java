package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;

public class HoodAndShooterPassing extends Command {
    private final HoodSubsystem m_HoodSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;
    private final SwerveSubsystem drivebase;

    public HoodAndShooterPassing(HoodSubsystem m_HoodSubsystem, ShooterSubsystem m_shShooterSubsystem,
            SwerveSubsystem swervedrive) {
        this.m_HoodSubsystem = m_HoodSubsystem;
        this.m_ShooterSubsystem = m_shShooterSubsystem;
        this.drivebase = swervedrive;
        addRequirements(m_HoodSubsystem, m_shShooterSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        String location = drivebase.getFieldLocation();
        if (location.equals("AllianceZone")) {

        } else if (location.equals("NeutralZone")) {
            m_HoodSubsystem.setHoodAngle(42.5);
            m_ShooterSubsystem.setShooterSpeed(() -> 4000.);

        } else if (location.equals("OpponentZone")) {
            m_HoodSubsystem.setHoodAngle(42.5);
            m_ShooterSubsystem.setShooterSpeed(() -> 6000.);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
};