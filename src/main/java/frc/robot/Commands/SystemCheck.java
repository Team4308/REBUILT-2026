
package frc.robot.Commands;

import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import frc.robot.Subsystems.LedSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

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
                        SwerveSubsystem swerveSubsystem, LedSubsystem ledSubsystem, ShooterSubsystem shooterSubsystem, 
                                        TurretSubsystem turretSubsystem) {
                addCommands(
                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 5, () -> 0, () -> 0)),

                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 0, () -> 5, () -> 0)),

                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 0, () -> 0, () -> 5)),

                                new WaitCommand(1),
                                new InstantCommand(() -> swerveSubsystem.lock()),

                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                hoodSubsystem.moveHood(() -> 15.0, 5.0)),
                                new WaitCommand(1),
                                new InstantCommand(() -> hoodSubsystem.resetHood()),

                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                intakeSubsystem.moveIntakeToAngle(25.0),
                                                intakeSubsystem.setRollerSpeed(() -> 15.0)),
                                new WaitCommand(1),
                                new InstantCommand(() -> intakeSubsystem.resetIntake()),
                                new InstantCommand(() -> intakeSubsystem.stopRoller()),
                                
                                new ParallelDeadlineGroup(new WaitCommand(1), 
                                                turretSubsystem.moveToTarget(() -> 15.0)),
                                new InstantCommand(() -> turretSubsystem.resetTurretCommand()),

                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                shooterSubsystem.setShooterSpeed(() -> 10.0)),
                                new WaitCommand(1),
                                new InstantCommand(() -> shooterSubsystem.setShooterSpeed(() -> 0.0)),

                                new ParallelDeadlineGroup(new WaitCommand(1), 
                                               indexerSubsystem.feedBalls())

                                );
        
        }
}








