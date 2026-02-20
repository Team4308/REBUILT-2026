// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.HoodSubsystem;

public class RobotContainer {

  public HoodSubsystem m_HoodSubsystem;

  private final XBoxWrapper m_driverController = new XBoxWrapper(Constants.Hood.kDriverControllerPort);

  public RobotContainer() {
    m_HoodSubsystem = new HoodSubsystem();
    configureBindings();
  }

  private void configureBindings() {
      // Toggle shoot/rest
      m_driverController.A.onTrue(
        Commands.runOnce(() -> {
            m_HoodSubsystem.setState(
                m_HoodSubsystem.getState() == HoodSubsystem.RobotState.SHOOT
                    ? HoodSubsystem.RobotState.REST
                    : HoodSubsystem.RobotState.SHOOT
            );
        })
      );

      // Pass presets
      m_driverController.X.onTrue(
          Commands.runOnce(() -> m_HoodSubsystem.setState(HoodSubsystem.RobotState.PASS_LEFT))
      );

      m_driverController.Y.onTrue(
          Commands.runOnce(() -> m_HoodSubsystem.setState(HoodSubsystem.RobotState.PASS_RIGHT))
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

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
