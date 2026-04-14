package frc.robot.Auton.Routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldLayout;
import frc.robot.Auton.Auto;
import frc.robot.Auton.AutoProgram;
import frc.robot.Commands.ShootAndAgitate254;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class MiddleDepotAuto extends AutoProgram {
    public MiddleDepotAuto(SwerveSubsystem drivebase, IndexerSubsystem m_indexerSubsystem,
            IntakeSubsystem m_intakeSubsystem) {
        super(Auto.MIDDEPOT, "Middle Depot", drivebase, FieldLayout.StartingPoses.getMidStartingPose(),
                drivebase.driveToPoseObjAvoid(new Pose2d(3.5, 4, new Rotation2d()), 0, 6),
                new ShootAndAgitate254(m_intakeSubsystem, m_indexerSubsystem));
    }
}