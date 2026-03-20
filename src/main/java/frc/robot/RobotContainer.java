package frc.robot;

import java.io.File;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import ca.team4308.absolutelib.control.RazerWrapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.AimAtPoseCommand;
import frc.robot.Commands.AimHoodAndTurret;
import frc.robot.Commands.DefaultIntakeCommand;
import frc.robot.Commands.HoodAndShooterPassing;
import frc.robot.Commands.IndexerCommand;
import frc.robot.Commands.MoveHoodCommand;
import frc.robot.Commands.MoveTurretCommand;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.SystemCheck;
import frc.robot.Commands.Trajectories.Shoot;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LedSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.Simulation;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Util.LaunchCalculator;
import swervelib.SwerveInputStream;

public class RobotContainer {
        // Controllers
        final RazerWrapper driver = new RazerWrapper(0);

        // Subsystems
        private final Vision vision = new Vision();
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));

        private final HoodSubsystem m_HoodSubsystem;
        private final IntakeSubsystem m_IntakeSubsystem;
        private final TurretSubsystem m_TurretSubsystem;
        private final IndexerSubsystem m_IndexerSubsystem;
        private final ShooterSubsystem m_ShooterSubsystem;
        private final LedSubsystem m_LedSubsystem;
        @SuppressWarnings("unused")
        private Simulation m_Simulation = null;

        // Commands
        private final SendableChooser<Command> autoChooser;

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driver.getLeftY() * -1,
                        () -> driver.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driver.getRightX())
                        .deadband(Constants.OperatorConstants.DEADBAND)
                        .scaleTranslation(1.0)
                        .allianceRelativeControl(true);

        // Clone's the angular velocity input stream and converts it to a fieldRelative
        // input stream.

        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driver::getRightX,
                                        driver::getRightY)
                        .headingWhile(true);

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

        Pose2d targetPoseForTESTING = new Pose2d(2, 4, new Rotation2d());

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */

        public RobotContainer() {
                drivebase.setVision(vision);

                m_HoodSubsystem = new HoodSubsystem(true);
                m_IndexerSubsystem = new IndexerSubsystem(true);
                m_TurretSubsystem = new TurretSubsystem(true);
                m_ShooterSubsystem = new ShooterSubsystem(true);
                m_IntakeSubsystem = new IntakeSubsystem(false);
                m_LedSubsystem = new LedSubsystem();

                if (Robot.isSimulation())
                        m_Simulation = new Simulation(m_HoodSubsystem, m_IndexerSubsystem, m_IntakeSubsystem,
                                        m_ShooterSubsystem, m_TurretSubsystem, drivebase);

                m_HoodSubsystem.setTurretSupplier(() -> m_TurretSubsystem.getAngleWrapped());

                m_IntakeSubsystem.setDefaultCommand(
                                new DefaultIntakeCommand(m_IntakeSubsystem, () -> driver.getRightTrigger(),
                                                () -> Constants.Intake.ROLLER_INTAKE_RPM));

                m_TurretSubsystem.setDefaultCommand(
                                new AimHoodAndTurret(m_HoodSubsystem, m_ShooterSubsystem, m_TurretSubsystem,
                                                () -> drivebase.getPose(), () -> drivebase.getRobotVelocity(),
                                                drivebase,
                                                () -> driver.RB.getAsBoolean(), () -> driver.LB.getAsBoolean()));

                // m_IndexerSubsystem.setDefaultCommand(m_IndexerSubsystem.preLoadBalls());

                configureNamedCommands();
                configureBindings();

                DriverStation.silenceJoystickConnectionWarning(true);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureBindings() {
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                driver.RB.whileTrue(new ShootCommand(m_IndexerSubsystem));
                driver.LB.whileTrue(new ShootCommand(m_IndexerSubsystem));

                // driver.M6.whileTrue(driveRobotOrientedAngularVelocity); // TODO: Could be
                // switched
                driver.M4.onTrue(new InstantCommand(() -> drivebase.lock())); // TODO: Could be switched

                // Reset Gyro
                driver.M1.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0,
                                0, new Rotation2d()))));

                // Reset Hood and Intake
                driver.M2.onTrue(m_HoodSubsystem.resetHoodCommand());
                // driver.M2.onTrue(m_IntakeSubsystem.resetIntakeCommand());

                // Full Auto Shooting
                // driver.LB.whileTrue(shootLeft);
                // driver.RB.whileTrue(shootRight);

                if (Robot.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
                }
        }

        public void configureNamedCommands() {
        }

        public Command getTestCommand() {
                return new SystemCheck(m_HoodSubsystem, m_IndexerSubsystem, m_IntakeSubsystem, drivebase,
                                m_LedSubsystem, m_ShooterSubsystem, m_TurretSubsystem);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        /**
         * Called each robot loop to update any periodic non-command logic.
         */
        public void periodic() {

        }
}
