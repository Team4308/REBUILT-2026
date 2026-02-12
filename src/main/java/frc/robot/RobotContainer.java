package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
  public final SwerveSubsystem swerve = new SwerveSubsystem(); 
  public final Vision vision = new Vision(swerve);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Bindings here
  }

  public Command getAutonomousCommand() {
    return Commands.print("No auto configured");
  }
}