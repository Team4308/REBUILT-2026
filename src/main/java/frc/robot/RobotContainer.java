package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();


  private double targetSpeed = 0;

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    // NamedCommands.registerCommand("Shoot", m_turretSubsystem.aimAtHub(),
    // m_HoodSubsystem.setState(HoodSubsystem.RobotState.SHOOT),m_shooterSubsystem.setState(ShooterSubsystem.ShooterState.SHOOTING));

    // Default command: right trigger controls shooter speed continuously
    m_shooterSubsystem.setDefaultCommand(m_shooterSubsystem.setShooterSpeed(() -> targetSpeed));

    m_driverController.a().whileTrue(m_shooterSubsystem.setShooterSpeed(() -> 0.0));

    m_driverController.b().whileTrue(m_shooterSubsystem.setShooterSpeed(() -> 1000.0));
    m_driverController.x().whileTrue(m_shooterSubsystem.setShooterSpeed(() -> 3000.0));
    m_driverController.y().whileTrue(m_shooterSubsystem.setShooterSpeed(() -> 5500.0));
  }

  public void periodic() {
    targetSpeed -= m_driverController.getLeftY() * 10;

    targetSpeed = MathUtil.clamp(
        targetSpeed,
        0.0,
        Constants.Shooter.kMaxRPM);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}