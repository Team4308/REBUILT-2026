package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShooterCommand;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final ShooterCommand m_shooterCommand = new ShooterCommand(
    m_shooterSubsystem,
    () -> m_driverController.getRightTriggerAxis() * Constants.Shooter.kMaxRPM
  );

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    // Default command: right trigger controls shooter speed continuously
    m_shooterSubsystem.setDefaultCommand(m_shooterCommand);

    // Hold A: spin up to hub speed
    m_driverController.a()
        .whileTrue(m_shooterCommand.setShooterSpeedHubCommand())
        .onFalse(Commands.runOnce(() -> m_shooterSubsystem.stopMotors(), m_shooterSubsystem));

    // Hold B: spin up to pass speed
    m_driverController.b()
        .whileTrue(m_shooterCommand.setShooterSpeedPassCommand())
        .onFalse(Commands.runOnce(() -> m_shooterSubsystem.stopMotors(), m_shooterSubsystem));

    // Hold X: spin up to a custom RPM with a 3 second timeout
    m_driverController.x()
        .whileTrue(m_shooterCommand.setShooterSpeed(
            () -> Constants.Shooter.kMaxRPM, 3.0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}