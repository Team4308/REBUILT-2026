package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.RazerWrapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.AimEverything;
import frc.robot.Commands.BackupShoot;
import frc.robot.Commands.BackupShootGroup;
import frc.robot.Commands.DefaultIntakeCommand;
import frc.robot.Commands.ShootAndAgitate254;
import frc.robot.Commands.ShootAndAgitateJ;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.SystemCheck;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.Simulation;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import frc.robot.Subsystems.vision.Vision;
import swervelib.SwerveInputStream;

public class RobotContainer {
        private double speedModifier = 1.0;

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
        @SuppressWarnings("unused")
        private Simulation m_Simulation = null;

        // Commands
        private final SendableChooser<Command> autoChooser;

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driver.getLeftY() * -1 * speedModifier,
                        () -> driver.getLeftX() * -1 * speedModifier)
                        .withControllerRotationAxis(() -> driver.getRightX())
                        .deadband(Constants.OperatorConstants.DEADBAND)
                        .scaleTranslation(1.0)
                        .allianceRelativeControl(true);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */

        public RobotContainer() {
                drivebase.setVision(vision);

                m_HoodSubsystem = new HoodSubsystem(true);
                m_IndexerSubsystem = new IndexerSubsystem(true);
                m_TurretSubsystem = new TurretSubsystem(true);
                m_ShooterSubsystem = new ShooterSubsystem(true);
                m_IntakeSubsystem = new IntakeSubsystem(true);

                if (Robot.isSimulation())
                        m_Simulation = new Simulation(m_HoodSubsystem, m_IndexerSubsystem, m_IntakeSubsystem,
                                        m_ShooterSubsystem, m_TurretSubsystem, drivebase);

                m_HoodSubsystem.setTurretSupplier(() -> m_TurretSubsystem.getAngleWrapped());

                m_IntakeSubsystem.setDefaultCommand(
                                new DefaultIntakeCommand(m_IntakeSubsystem, () -> driver.getRightTrigger(),
                                                () -> Constants.Intake.ROLLER_INTAKE_RPM));

                m_TurretSubsystem.setDefaultCommand(
                                new AimEverything(m_HoodSubsystem, m_ShooterSubsystem, m_TurretSubsystem,
                                                () -> drivebase.getPose(), () -> drivebase.getRobotVelocity(),
                                                drivebase,
                                                () -> driver.RB.getAsBoolean() || driver.RightTrigger.getAsBoolean(),
                                                () -> driver.LB.getAsBoolean() || driver.LeftTrigger.getAsBoolean()));

                configureNamedCommands();
                configureBindings();

                DriverStation.silenceJoystickConnectionWarning(true);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureBindings() {
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                driver.RB.whileTrue(new ShootCommand(m_IndexerSubsystem));
                driver.RB.onTrue(m_ShooterSubsystem.setShooterSpeed(() -> 2800.));
                driver.LB.whileTrue(new ShootCommand(m_IndexerSubsystem));
                driver.LB.onTrue(m_ShooterSubsystem.setShooterSpeed(() -> 3600.));

                driver.X.whileTrue(new BackupShoot(m_HoodSubsystem, m_TurretSubsystem, m_ShooterSubsystem,
                                drivebase));

                driver.M1.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0,
                                0, new Rotation2d()))));

                driver.M2.onTrue(m_HoodSubsystem.resetHoodCommand());
                driver.M2.onTrue(m_IntakeSubsystem.resetIntakeCommand());

                driver.RB.onTrue(Commands.runOnce(() -> speedModifier = 0.5));
                driver.RB.onFalse(Commands.runOnce(() -> speedModifier = 1.0));
                driver.LB.onTrue(Commands.runOnce(() -> speedModifier = 0.5));
                driver.LB.onFalse(Commands.runOnce(() -> speedModifier = 1.0));
        }

        public void configureNamedCommands() {
                NamedCommands.registerCommand("Shoot", new ShootCommand(m_IndexerSubsystem));
                NamedCommands.registerCommand("Shoot Agitate J",
                                new ShootAndAgitateJ(m_IntakeSubsystem, m_IndexerSubsystem));
                NamedCommands.registerCommand("Shoot Agitate 254",
                                new ShootAndAgitate254(m_IntakeSubsystem, m_IndexerSubsystem));
                NamedCommands.registerCommand("Move Away", drivebase.driveToPoseObjAvoid(
                                () -> new Pose2d(3.5, 4, new Rotation2d(Units.degreesToRadians(180)))));
                NamedCommands.registerCommand("Agitate", m_IntakeSubsystem.agitate());
        }

        public Command getTestCommand() {
                return new SystemCheck(m_HoodSubsystem, m_IndexerSubsystem, m_IntakeSubsystem, drivebase,
                                m_ShooterSubsystem, m_TurretSubsystem);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
