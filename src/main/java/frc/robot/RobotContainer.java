// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.turretSubsystem;

public class RobotContainer {
  public turretSubsystem m_turret = new turretSubsystem();


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //add button bindings here
  } 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  
  public void robotPeriodic() {
    m_turret.setTarget(720.00);
  }
}
