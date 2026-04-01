package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.Intake.MoveIntakeTimed;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class ShootAndAgitate254 extends ParallelCommandGroup {
    public ShootAndAgitate254(IntakeSubsystem m_IntakeSubsystem, IndexerSubsystem m_IndexerSubsystem) {
        addCommands(
                new ShootCommand(m_IndexerSubsystem),
                new MoveIntakeTimed(m_IntakeSubsystem, Constants.Intake.RETRACTED_ANGLE_DEG, 5));
    }
}