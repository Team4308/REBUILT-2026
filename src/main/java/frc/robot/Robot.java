package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.FuelSim;
import frc.robot.Util.GameData;
import frc.robot.Subsystems.swerve.LocalADStarAK;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    Pathfinding.setPathfinder(new LocalADStarAK()); // Uses OBJ avoidance PathFinder

    Logger.recordMetadata("REBUILT-2026", "FRC-4308"); // Set a metadata value

    Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    Logger.recordOutput("Match Timer", DriverStation.getMatchTime());
    Logger.recordOutput("Ally Hub Active", GameData.isHubActive(false));
    Logger.recordOutput("Opposing Hub Active", GameData.isHubActive(true));
    Logger.recordOutput("Time To Next Phase", GameData.timeToNextPhase());

    // Ensure trajectory logging runs in all modes (real + sim).
    m_robotContainer.periodic();

    HttpCamera backCamera = new HttpCamera(
        "ExternalCam",
        "http://10.43.8.11:1181", // is this right?
        HttpCamera.HttpCameraKind.kMJPGStreamer);

    CameraServer.startAutomaticCapture(backCamera);
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  public void teleopPeriodic() {
    // No-op: periodic work is handled in robotPeriodic so it runs in all modes.
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {

  }

  @Override
  public void simulationPeriodic() {
    FuelSim.getInstance().updateSim();
    // Update Logger
    m_robotContainer.getTrajectoryCalculations().periodic();
    Logger.recordOutput("Moving Pose",
        new Pose3d(0, 0, 0, new Rotation3d(0, 2.0 * Math.sin(2 * Math.PI * Timer.getFPGATimestamp() / 3.0), 0)));
    Logger.recordOutput("Zeroed Pose", new Pose3d());
  }
}
