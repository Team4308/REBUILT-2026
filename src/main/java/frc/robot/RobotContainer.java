// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.intakeSubsystem;

public class RobotContainer {
  private final XBoxWrapper m_driverController = new XBoxWrapper(Constants.kDriverControllerPort);

  public intakeSubsystem m_IntakeSubsystem;

  double targetAngle;
  double targetRPM;

  public RobotContainer() {
    targetAngle = 0.0;
    targetRPM = 0.0;
    m_IntakeSubsystem = new intakeSubsystem();
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.A.onTrue(new InstantCommand(() -> targetRPM = 0 ));
    m_driverController.X.onTrue(new InstantCommand(() -> targetRPM = 1000 ));
    m_driverController.Y.onTrue(new InstantCommand(() -> targetRPM = 2000 ));
    m_driverController.B.onTrue(new InstantCommand(() -> targetRPM = 3000 ));
  }

  public void periodic() {
    // Manual control for testing, won't work with bindings cuz locally saved variables why
    targetAngle += m_driverController.getLeftY();
    targetRPM += m_driverController.getRightY();
    targetAngle = MathUtil.clamp(targetAngle, Constants.Intake.RETRACTED_ANGLE_DEG, Constants.Intake.INTAKE_ANGLE_DEG);
   // m_IntakeSubsystem.setIntakeAngle(targetAngle);
    m_IntakeSubsystem.setRollerSpeed(targetRPM);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
