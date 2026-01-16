// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import ca.team4308.absolutelib.control.RazerWrapper;

public class RobotContainer {
  // Controllers
  final RazerWrapper driver = new RazerWrapper(0);

  // Subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  // Commands
  private final SendableChooser<Command> autoChooser;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driver.getLeftY() * -1,
      () -> driver.getLeftX() * -1)
      .withControllerRotationAxis(() -> driver.getRightX() * -1)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(1.0)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX,
      driver::getRightY)
      .headingWhile(true);

  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driver.getLeftY(),
      () -> -driver.getLeftX())
      .withControllerRotationAxis(() -> driver.getLeftTrigger())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureNamedCommands();
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    driver.M1.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));
    driver.B.whileTrue(
        drivebase.driveToPose(
            new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
  }
  
  public void configureNamedCommands() {

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}