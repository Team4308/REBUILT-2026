package frc.robot;

import ca.team4308.absolutelib.control.RazerWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.MoveHoodCommand;
import frc.robot.Commands.MoveTurretCommand;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

public class RobotContainer {
        // Controllers
        final RazerWrapper driver = new RazerWrapper(0);

        private final TurretSubsystem m_turretSubsystem;
        private final HoodSubsystem m_HoodSubsystem;
        private final IndexerSubsystem m_IndexerSubsystem;

        public RobotContainer() {
                m_turretSubsystem = new TurretSubsystem(true);
                m_HoodSubsystem = new HoodSubsystem(true);
                m_IndexerSubsystem = new IndexerSubsystem(true);

                configureBindings();
        }

        private void configureBindings() {
                driver.povRight.onTrue(new MoveTurretCommand(m_turretSubsystem, () -> 5.));
                driver.povLeft.onTrue(new MoveTurretCommand(m_turretSubsystem, () -> -5.));
                // driver.A.whileTrue(m_IndexerSubsystem.preLoadBalls())
        }

        public void configureNamedCommands() {
        }

        public Command getAutonomousCommand() {
                return null;
        }

}