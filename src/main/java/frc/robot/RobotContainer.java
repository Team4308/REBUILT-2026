// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
  private final XBoxWrapper m_driverController = new XBoxWrapper(Constants.kDriverControllerPort);

  public IntakeSubsystem m_IntakeSubsystem;

  double targetAngle;
  double targetRPM;

  public RobotContainer() {
    targetAngle = 0.0;
    targetRPM = 0.0;
    m_IntakeSubsystem = new IntakeSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.A.onTrue(m_IntakeSubsystem.intake());
    m_driverController.X.onTrue(m_IntakeSubsystem.retract());
    m_driverController.Y.onTrue(m_IntakeSubsystem.retract(1000));
    m_driverController.B.onTrue(m_IntakeSubsystem.agitate());
  }

  public void periodic() {
    // Manual control for testing, won't work with bindings cuz locally saved variables why
    targetAngle += m_driverController.getLeftY();
    targetRPM += m_driverController.getRightY();
    targetAngle = MathUtil.clamp(targetAngle, Constants.Intake.RETRACTED_ANGLE_DEG, Constants.Intake.INTAKE_ANGLE_DEG);
    m_IntakeSubsystem.setIntakeAngle(targetAngle);
    m_IntakeSubsystem.setRollerSpeed(targetRPM);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
