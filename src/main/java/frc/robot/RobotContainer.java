package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommand;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ShooterCommand m_shooterCommand = new ShooterCommand(
      m_shooterSubsystem,
      () -> m_driverController.getRightTriggerAxis() * Constants.Shooter.kMaxRPM);

  private double targetSpeed = 0;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // NamedCommands.registerCommand("Shoot", m_turretSubsystem.aimAtHub(),
    // m_HoodSubsystem.setState(HoodSubsystem.RobotState.SHOOT),m_shooterCommand.setState(ShooterSubsystem.ShooterState.SHOOTING));

    // Default command: right trigger controls shooter speed continuously
    m_shooterSubsystem.setDefaultCommand(m_shooterCommand);

    m_driverController.a().onTrue(new InstantCommand(() -> targetSpeed = 0));

    m_driverController.b().onTrue(new InstantCommand(() -> targetSpeed = 1000));

    m_driverController.x().onTrue(new InstantCommand(() -> targetSpeed = 3000));

    m_driverController.y().onTrue(new InstantCommand(() -> targetSpeed = 1500));
  }

  public void periodic() {
    targetSpeed -= m_driverController.getLeftY() * 10;

    targetSpeed = MathUtil.clamp(
        targetSpeed,
        0.0,
        Constants.Shooter.kMaxRPM);
    m_shooterSubsystem.setTargetSpeed(targetSpeed);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}