// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ca.team4308.absolutelib.control.RazerWrapper;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Util.FuelSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.io.File;
import java.util.ArrayList;

import swervelib.SwerveInputStream;

public class RobotContainer {
  // Controllers
  final RazerWrapper driver = new RazerWrapper(0);

  // Subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // Commands
  private final SendableChooser<Command> autoChooser;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driver.getLeftY() * -1,
      () -> driver.getLeftX() * -1)
      .withControllerRotationAxis(() -> driver.getRightX() * -1)
      .deadband(Constants.OperatorConstants.DEADBAND)
      .scaleTranslation(1.0)
      .allianceRelativeControl(true);

  // Clone's the angular velocity input stream and converts it to a fieldRelative
  // input stream.
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX,
      driver::getRightY)
      .headingWhile(true);

  // Clone's the angular velocity input stream and converts it to a roboRelative
  // input stream.
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driver.getLeftY(),
      () -> -driver.getLeftX())
      .withControllerRotationAxis(() -> driver.getRightX())
      .deadband(Constants.OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driver.getLeftTrigger() * Math.PI) * (Math.PI * 2),
          () -> Math.cos(driver.getLeftTrigger() * Math.PI) * (Math.PI * 2))
      .headingWhile(true);

  // Command lists
  ArrayList<Command> movementCommands = new ArrayList<>();
  ArrayList<Command> actionCommands = new ArrayList<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureNamedCommands();
    configureBindings();
    initFuelSim();

    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);

    driver.M1.onTrue(
        Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));

    driver.LeftTrigger.whileTrue(driveRobotOrientedAngularVelocity);

    driver.M3.whileTrue(drivebase.moveUpLeft());
    driver.M4.whileTrue(drivebase.moveUpRight());
    driver.M5.whileTrue(drivebase.moveDownLeft());
    driver.M6.whileTrue(drivebase.moveDownRight());

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driver.X.onTrue(
        Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(2, 4, new Rotation2d()))));

    driver.Y.whileTrue(driveFieldOrientedAnglularVelocityKeyboard);
    driver.A.onTrue(drivebase.driveToPoseObjAvoid(() -> new Pose2d(2, 4, new Rotation2d())));
  }

  public void configureNamedCommands() {
  }

  public Command getAutonomousCommand() {
    /*
     * rows are slots: actions that happen in sequence
     * 2 cols are actions: movement + action
     * leave blank to say no action
     * this way we can just chain commands together as long as they are able to be
     * chained together
     */

    ArrayList<SendableChooser<Command>> movementChoosers = new ArrayList<>();
    ArrayList<SendableChooser<Command>> actionChoosers = new ArrayList<>();

    for (int i = 0; i < Constants.Auton.maxLength; i++) {
      movementChoosers.add(i, new SendableChooser<Command>());
      for (SendableChooser<Command> chooser : movementChoosers) {
        for (Command toAdd : movementCommands)
          chooser.addOption("String", toAdd);
        chooser.setDefaultOption("String", null);
      }

      actionChoosers.add(i, new SendableChooser<Command>());
      for (SendableChooser<Command> chooser : actionChoosers) {
        for (Command toAdd : actionCommands)
          chooser.addOption("String", toAdd);
        chooser.setDefaultOption("String", null);
      }
    }

    // If we select a premade auton, we will run it instead
    if (autoChooser.getSelected() != null) {
      return autoChooser.getSelected();
    }

    // TODO: figure out how to link commands together. skip commands if null? i
    // guess
    return Commands.sequence(
        new InstantCommand(() -> drivebase.setRobotPose(new Pose2d(2, 2, new Rotation2d()))),
        drivebase.driveToPoseObjAvoid(() -> new Pose2d(2, 6, new Rotation2d(0))),
        drivebase.driveToPoseObjAvoid(() -> new Pose2d(2, 2, new Rotation2d(Units.degreesToRadians(90)))),
        drivebase.driveToPoseObjAvoid(() -> new Pose2d(2, 4, new Rotation2d(0))),
        drivebase.driveToPoseObjAvoid(() -> new Pose2d(7, 7.2, new Rotation2d(Units.degreesToRadians(180)))),
        drivebase.driveToPoseObjAvoid(() -> new Pose2d(7.5, 3, new Rotation2d(0))),
        drivebase.driveToPoseObjAvoid(() -> new Pose2d(2.6, 1, new Rotation2d(Units.degreesToRadians(270)))),
        drivebase.driveToPoseObjAvoid(() -> new Pose2d(14, 4, new Rotation2d(Units.degreesToRadians(67)))),
        drivebase.driveToPoseObjAvoid(() -> new Pose2d(10, 4, new Rotation2d(0))));

  }

  private void initFuelSim() {
    FuelSim.getInstance(); // gets singleton instance of FuelSim
    FuelSim.getInstance().spawnStartingFuel(); // spawns fuel in the depots and neutral zone

    // Register a robot for collision with fuel
    FuelSim.getInstance().registerRobot(
        Units.inchesToMeters(34), // from left to right
        Units.inchesToMeters(34), // from front to back
        Units.inchesToMeters(6), // from floor to top of bumpers
        drivebase::getPose, // Supplier<Pose2d> of robot pose
        drivebase::getFieldVelocity); // Supplier<ChassisSpeeds> of field-centric chassis speeds

    FuelSim.getInstance().setSubticks(5); // sets the number of physics iterations to perform per 20ms loop. Default = 5

    FuelSim.getInstance().start();
  }
}
