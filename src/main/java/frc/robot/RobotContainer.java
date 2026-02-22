// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.turretSubsystem;

import ca.team4308.absolutelib.control.XBoxWrapper;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.RobotState;

public class RobotContainer {
  public turretSubsystem m_turret = new turretSubsystem();


  public RobotContainer() {
    configureBindings();
  }

    private final Joystick driver = new Joystick(0);

    double x = driver.getRawAxis(0); 
    private final JoystickButton aButton = new JoystickButton(driver, 1);
    private final JoystickButton bButton = new JoystickButton(driver, 2);
    private final JoystickButton xButton = new JoystickButton(driver, 3);
  
  private void configureBindings() {

  aButton.onTrue(
      new InstantCommand(() -> m_turret.switchState(RobotState.aimAtHub))
  );

  bButton.onTrue(
      new InstantCommand(() -> m_turret.switchState(RobotState.aimAtPassingZone))
  );

  xButton.onTrue(
      new InstantCommand(() -> m_turret.switchState(RobotState.defaultTurret))
  );
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  
  public void robotPeriodic() {
    m_turret.setTarget(720.00);
  }
}
