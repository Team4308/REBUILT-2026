package frc.robot;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HoodSubsystem;

public class RobotContainer {

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  }

  public void periodic() {
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
