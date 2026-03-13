package frc.robot;

import java.io.File;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import ca.team4308.absolutelib.control.RazerWrapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.AimAtHubCommand;
import frc.robot.Commands.MoveHoodCommand;
import frc.robot.FieldLayout;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Commands.TriggerIntakeCommand;
import frc.robot.Commands.MoveTurretCommand;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LedSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.Simulation;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import frc.robot.Subsystems.vision.Vision;
import frc.robot.Util.FuelSim;
import frc.robot.Util.TrajectoryCalculations;
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
        private Simulation m_Simulation = null;

        private double m_hoodAngle = 7.5;
        private double m_turretAngle = 180;
        private double m_shooterSpeed = 0.0;
        private double m_indexerSpeed = 0.0;

        private TrajectoryCalculations m_TrajectoryCalculations;

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
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driver::getRightX, driver::getRightY).headingWhile(true);

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

        Pose2d targetPoseForTESTING = new Pose2d(2, 4, new Rotation2d());

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */

        public RobotContainer() {
                drivebase.setVision(vision);

                m_HoodSubsystem = new HoodSubsystem();
                m_IndexerSubsystem = new IndexerSubsystem();
                m_TurretSubsystem = new TurretSubsystem();
                m_ShooterSubsystem = new ShooterSubsystem();
                m_IntakeSubsystem = new IntakeSubsystem();
                m_LedSubsystem = new LedSubsystem();

                if (Robot.isSimulation())
                        m_Simulation = new Simulation(m_HoodSubsystem, m_IndexerSubsystem, m_IntakeSubsystem,
                                        m_ShooterSubsystem, m_TurretSubsystem, drivebase);

                m_HoodSubsystem.setTurretSupplier(() -> m_TurretSubsystem.getAngleWrapped());

                // m_IntakeSubsystem.setDefaultCommand(
                // new TriggerIntakeCommand(m_IntakeSubsystem, () -> driver.getRightTrigger()));

                configureNamedCommands();
                configureBindings();

                DriverStation.silenceJoystickConnectionWarning(true);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                m_TrajectoryCalculations = new TrajectoryCalculations();
                m_TrajectoryCalculations.setChassisSupplier(() -> drivebase.getFieldVelocity());
                m_TrajectoryCalculations.setCurrentRPMsupply(() -> m_ShooterSubsystem.getRPM());
                m_TrajectoryCalculations.setPoseSupplier(() -> drivebase.getPose());
                m_TrajectoryCalculations.setTargetSupplier(() -> FieldLayout.ShooterTargets.getAllianceHub());
                m_HoodSubsystem.setTrajectoryCalculations(m_TrajectoryCalculations);
        }

        private void configureBindings() {
                /*
                 * Joysticks: swerve
                 * POV: Hood & Turret Increments
                 * X: Use Auto Targeting
                 * Y: Extend / Retract Intake
                 * A: Shooter at fixed speed a
                 * B: Shooter at fixed speed b
                 * Left Trigger: NOTHING
                 * LB: Intake
                 * Left Small Button: Reset Pose Odometry
                 * Right Trigger: NOTHING
                 * RB: Shoot
                 * Right Small Button: Reset Hood
                 */

                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                driver.povUp.onTrue(new MoveHoodCommand(m_HoodSubsystem, () -> 5.));
                driver.povDown.onTrue(new MoveHoodCommand(m_HoodSubsystem, () -> -5.));

                driver.povRight.onTrue(new MoveTurretCommand(m_TurretSubsystem, () -> 5.));
                driver.povLeft.onTrue(new MoveTurretCommand(m_TurretSubsystem, () -> -5.));

                // driver.X.whileTrue(new HoodCommand(m_HoodSubsystem, () ->
                // m_TrajectoryCalculations.getNeededPitch()));
                // driver.X.whileTrue(new TurretCommand(m_TurretSubsystem, () ->
                // m_TrajectoryCalculations.getNeededYaw()));
                // driver.X.whileTrue(new ShooterCommand(m_ShooterSubsystem, () ->
                // m_TrajectoryCalculations.getNeededRPM()));
                // driver.X.whileTrue(m_TurretSubsystem.aimAtPointCommand(FieldLayout.ShooterTargets.kHUB_POSE));
                // driver.X.whileTrue(new RunCommand(null, null));

                // Semi Auto Shooting

                driver.X.whileTrue(new AimAtHubCommand(() -> drivebase.getPose(), m_TurretSubsystem));
                driver.A.whileTrue(m_ShooterSubsystem.setShooterSpeed(() -> 3000.));
                driver.A.onFalse(new InstantCommand(() -> m_ShooterSubsystem.stopMotors()));
                driver.B.whileTrue(m_ShooterSubsystem.setShooterSpeed(() -> 2300.));
                driver.B.onFalse(new InstantCommand(() -> m_ShooterSubsystem.stopMotors()));

                driver.X.onTrue(new InstantCommand(() -> m_HoodSubsystem.setHoodAngle(7.5)));
                driver.Y.onTrue(new InstantCommand(() -> m_HoodSubsystem.setHoodAngle(15)));
                driver.B.onTrue(new InstantCommand(() -> m_HoodSubsystem.setHoodAngle(32)));
                driver.A.onTrue(new InstantCommand(() -> m_HoodSubsystem.setHoodAngle(52.5)));

                // Reset Gyro
                driver.M1.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0, 0, new Rotation2d()))));

                // Intaking
                driver.RB.whileTrue(new InstantCommand(() -> m_IndexerSubsystem.setIndexerVelocity(500)));
                driver.RB.whileTrue(new InstantCommand(() -> m_IndexerSubsystem.setHopperVelocity(500)));
                driver.RB.onFalse(new InstantCommand(() -> m_IndexerSubsystem.stopMotors()));

                driver.LB.onTrue(new InstantCommand(() -> m_IntakeSubsystem.setRollerSpeedA(() -> -100.)));
                driver.LB.onFalse(new InstantCommand(() -> m_IntakeSubsystem.stopMotors()));

                // Reset Hood and Intake
                driver.M2.onTrue(m_HoodSubsystem.resetHoodCommand());
                driver.M2.onTrue(m_IntakeSubsystem.resetIntakeCommand());

                // driver.LB.whileTrue(driveRobotOrientedAngularVelocity);

                if (Robot.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
                }

        }

        public void periodic() {
                m_TrajectoryCalculations.periodic();

        }

        public TrajectoryCalculations getTrajectoryCalculations() {
                return m_TrajectoryCalculations;
        }

        public void configureNamedCommands() {
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}