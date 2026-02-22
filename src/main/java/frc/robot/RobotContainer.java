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
import frc.robot.Constants.Driver;

import frc.robot.subsystems.indexer.IndexerSubsystem;

public class RobotContainer {

  private final XBoxWrapper driver = new XBoxWrapper(Ports.Joysticks.DRIVER);

  private final IndexerSubsystem indexer = new IndexerSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driver.A.onTrue(
        Commands.runOnce(indexer::runMotors, indexer)
  );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}