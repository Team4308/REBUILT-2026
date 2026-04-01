
package frc.robot.Commands;

import frc.robot.Commands.Hood.MoveHoodToAngle;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SystemCheck extends SequentialCommandGroup {
        public SystemCheck(HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem,
                        TurretSubsystem turretSubsystem) {
                addCommands(
                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 5, () -> 0, () -> 0)),

                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 0, () -> 5, () -> 0)),

                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 0, () -> 0, () -> 5)),

                                new InstantCommand(() -> swerveSubsystem.lock()),

                                new MoveHoodToAngle(hoodSubsystem, 42.),
                                new MoveHoodToAngle(hoodSubsystem, 7.5),

                                turretSubsystem.moveToTarget(() -> 500.),
                                turretSubsystem.moveToTarget(() -> 270.),
                                turretSubsystem.moveToTarget(() -> 360.),

                                new ParallelCommandGroup(new WaitCommand(1),
                                                shooterSubsystem.setShooterSpeed(() -> 3000)),
                                new ParallelCommandGroup(new WaitCommand(1), shooterSubsystem.setShooterSpeed(() -> 0)),

                                intakeSubsystem.moveIntakeToAngle(0),
                                intakeSubsystem.moveIntakeToAngle(127),

                                new ParallelCommandGroup(new WaitCommand(1),
                                                new InstantCommand(() -> intakeSubsystem.setRollerSpeed(() -> 3000.))),
                                new ParallelCommandGroup(new WaitCommand(1),
                                                new InstantCommand(() -> intakeSubsystem.setRollerSpeed(() -> 0.))),

                                new ParallelCommandGroup(new WaitCommand(1),
                                                new InstantCommand(
                                                                () -> indexerSubsystem.setIndexerVelocity(6000, 3000))),
                                new ParallelCommandGroup(new WaitCommand(1),
                                                new InstantCommand(() -> indexerSubsystem.setIndexerVelocity(0, 0))));

        }
}
