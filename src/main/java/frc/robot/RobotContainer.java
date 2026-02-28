// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.jar.Attributes.Name;

import com.ctre.phoenix.platform.can.AutocacheState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

import frc.robot.subsystems.IndexerSubsystem;

public class RobotContainer {

  private final XBoxWrapper driver = new XBoxWrapper(0);

  private final IndexerSubsystem indexer = new IndexerSubsystem();

  private double targetSpeedHopper = 0.0;
  private double targetSpeedIndexer = 0.0;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driver.A.onTrue(new InstantCommand(() -> targetSpeedHopper = 0.0));
    driver.B.onTrue(new InstantCommand(() -> targetSpeedIndexer = 0.0));
    driver.X.onTrue(new InstantCommand(() -> targetSpeedHopper = 100.0));
    driver.Y.onTrue(new InstantCommand(() -> targetSpeedIndexer = 100.0));
    driver.povUp.onTrue(new InstantCommand(() -> targetSpeedHopper = 500.0));
    driver.povDown.onTrue(new InstantCommand(() -> targetSpeedIndexer = 500.0));
  }

  public void periodic() {
    targetSpeedHopper -= driver.getLeftY() * 10;

    targetSpeedHopper = MathUtil.clamp(
        targetSpeedHopper,
        0.0,
        1900.0);
    indexer.setHopperSpeed(targetSpeedHopper);

    targetSpeedIndexer -= driver.getRightY() * 10;
    targetSpeedIndexer = MathUtil.clamp(
        targetSpeedIndexer,
        0.0,
        1900.0);
    indexer.setIndexerSpeed(targetSpeedIndexer);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
