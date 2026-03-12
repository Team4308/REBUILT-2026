
package frc.robot.Commands;

import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import frc.robot.Subsystems.LedSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Commands.AimAtHubCommand;
import frc.robot.Commands.IndexerCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.MoveHoodCommand;
import frc.robot.Commands.MoveTurretCommand;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Commands.TriggerIntakeCommand;


public class SystemCheck extends SequentialCommandGroup {
        public SystemCheck(HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem, IntakeSubsystem intakeSubsystem,
                        SwerveSubsystem swerveSubsystem, LedSubsystem ledSubsystem, ShooterSubsystem shootersubsystem, 
                                        TurretSubsystem turretsubsystem) {
                addCommands(
                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 5, () -> 0, () -> 0)),

                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 0, () -> 5, () -> 0)),

                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 0, () -> 0, () -> 5)),

                                new WaitCommand(1),
                                new InstantCommand(() -> swerveSubsystem.lock()),

                                new ParallelDeadlineGroup(new WaitCommand(1)),
                                                hoodSubsystem.moveHood(() -> 15, () -> 5),
                                new WaitCommand(1),
                                new InstantCommand(() -> hoodSubsytem.resetHood()),

                                new ParallelDeadlineGroup(new WaitCommand(1)),
                                                intakeSubsystem.setIntakeAngle(() -> 25),
                                                intakeSubsystem.setRollerSpeedA(() -> 15),
                                new WaitCommand(1),
                                new Instant
                )
        }
}








