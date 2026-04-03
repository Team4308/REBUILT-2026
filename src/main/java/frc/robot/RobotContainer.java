package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.Commands.ShootAndAgitate254;
import frc.robot.Commands.ShootAndAgitateJ;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.SystemCheck;
import frc.robot.Commands.Hood.ResetHood;
import frc.robot.Commands.Intake.AgitateJ;
import frc.robot.Commands.Intake.DefaultIntake;
import frc.robot.Commands.Intake.ResetIntake;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.Simulation;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Util.Razer2Wrapper;
import swervelib.SwerveInputStream;

public class RobotContainer {
        // Controllers
        final Razer2Wrapper driver = new Razer2Wrapper(0, 1);

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

        private final AimEverything aimEverythingCommand;

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driver.getLeftY() * -1,
                        () -> driver.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driver.getRightX() * -1)
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
                                new DefaultIntake(m_IntakeSubsystem, () -> driver.getRightTrigger(),
                                                () -> Constants.Intake.ROLLER_INTAKE_RPM));

                aimEverythingCommand = new AimEverything(m_IndexerSubsystem, m_HoodSubsystem, m_ShooterSubsystem,
                                m_TurretSubsystem,
                                () -> drivebase.getPose(), () -> drivebase.getRobotVelocity(),
                                drivebase);
                m_TurretSubsystem.setDefaultCommand(
                                aimEverythingCommand);

                configureNamedCommands();
                configureBindings();

                DriverStation.silenceJoystickConnectionWarning(true);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureBindings() {
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                driver.RB.whileTrue(new ShootCommand(m_IndexerSubsystem, m_TurretSubsystem, m_HoodSubsystem)
                                .alongWith(new InstantCommand(() -> aimEverythingCommand.setSide("Right"))));
                driver.LB.whileTrue(new ShootCommand(m_IndexerSubsystem, m_TurretSubsystem, m_HoodSubsystem)
                                .alongWith(new InstantCommand(() -> aimEverythingCommand.setSide("Left"))));

                driver.RightTrigger.whileTrue(new ShootAndAgitate254(m_IntakeSubsystem, m_IndexerSubsystem,
                                m_TurretSubsystem, m_HoodSubsystem,
                                () -> m_IntakeSubsystem.getHopperState())
                                .alongWith(new InstantCommand(() -> aimEverythingCommand.setSide("Right"))));
                driver.LeftTrigger.whileTrue(new ShootAndAgitate254(m_IntakeSubsystem, m_IndexerSubsystem,
                                m_TurretSubsystem, m_HoodSubsystem,
                                () -> m_IntakeSubsystem.getHopperState())
                                .alongWith(new InstantCommand(() -> aimEverythingCommand.setSide("Left"))));

                driver.povUp.onTrue(new InstantCommand(() -> aimEverythingCommand.updatePowerOffset(100)));
                driver.povDown.onTrue(new InstantCommand(() -> aimEverythingCommand.updatePowerOffset(-100)));

                driver.A.onTrue(new InstantCommand(
                                () -> m_IntakeSubsystem.setHopperState(IntakeSubsystem.HopperStates.EMPTY)));
                driver.X.onTrue(new InstantCommand(
                                () -> m_IntakeSubsystem.setHopperState(IntakeSubsystem.HopperStates.HALF)));
                driver.Y.onTrue(new InstantCommand(
                                () -> m_IntakeSubsystem.setHopperState(IntakeSubsystem.HopperStates.FULL)));

                driver.B.whileTrue(new BackupShoot(m_HoodSubsystem, m_TurretSubsystem, m_ShooterSubsystem,
                                drivebase));

                driver.M1.onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0,
                                0, new Rotation2d()))));

                driver.M2.onTrue(new ResetHood(m_HoodSubsystem));
                driver.M2.onTrue(new ResetIntake(m_IntakeSubsystem));
        }

        public void configureNamedCommands() {
                NamedCommands.registerCommand("Shoot",
                                new ShootCommand(m_IndexerSubsystem, m_TurretSubsystem, m_HoodSubsystem));
                NamedCommands.registerCommand("Shoot Agitate J",
                                new ShootAndAgitateJ(m_IntakeSubsystem, m_IndexerSubsystem, m_TurretSubsystem,
                                                m_HoodSubsystem));
                NamedCommands.registerCommand("Shoot Agitate 254",
                                new ShootAndAgitate254(m_IntakeSubsystem, m_IndexerSubsystem, m_TurretSubsystem,
                                                m_HoodSubsystem,
                                                () -> m_IntakeSubsystem.getHopperState()));
                NamedCommands.registerCommand("Move Away", drivebase.driveToPoseObjAvoid(
                                () -> new Pose2d(3.5, 4, new Rotation2d(Units.degreesToRadians(180)))));
                NamedCommands.registerCommand("Agitate", new AgitateJ(m_IntakeSubsystem));
                NamedCommands.registerCommand("Extend Intake", new InstantCommand(
                                () -> m_IntakeSubsystem.setIntakeAngle(Constants.Intake.INTAKE_ANGLE_DEG)));
        }

        public Command getTestCommand() {
                return new SystemCheck(m_HoodSubsystem, m_IndexerSubsystem, m_IntakeSubsystem, drivebase,
                                m_ShooterSubsystem, m_TurretSubsystem);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
