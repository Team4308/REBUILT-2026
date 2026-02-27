// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HoodSubsystem;

public class RobotContainer {

  public HoodSubsystem m_HoodSubsystem;

  private final XBoxWrapper m_driverController = new XBoxWrapper(Constants.Hood.kDriverControllerPort);
  double targetAngle = 8;
  public RobotContainer() {
    m_HoodSubsystem = new HoodSubsystem();
    configureBindings();
  }

  private void configureBindings() {
      // Toggle shoot/rest
      m_driverController.A.onTrue(
        new InstantCommand(()->m_HoodSubsystem.setHoodAngle(8))
      );

      // Pass presets
      m_driverController.X.onTrue(
          new InstantCommand(()->m_HoodSubsystem.setHoodAngle(53))
      );

      m_driverController.Y.onTrue(
          new InstantCommand(()->m_HoodSubsystem.setHoodAngle(30))
      );

      // Reset hood
      m_driverController.B.onTrue(
          m_HoodSubsystem.resetHoodCommand()
      );
      // Emergency stop
      m_driverController.Back.onTrue(
          Commands.runOnce(() -> m_HoodSubsystem.stopMotors())
      );
  }
  public void periodic(){
    targetAngle += m_driverController.getLeftY() * 2;
    targetAngle = MathUtil.clamp(
            targetAngle,
            Constants.Hood.REVERSE_SOFT_LIMIT_ANGLE,
            Constants.Hood.FORWARD_SOFT_LIMIT_ANGLE
        );
    
  }
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
