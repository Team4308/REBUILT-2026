package frc.robot.Commands.Trajectories;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Util.TrajectoryCalculations;

public class Shoot extends Command{
    private final IndexerSubsystem m_IndexerSubsystem;
    private final Supplier<String> fieldLocation;
    private final Translation3d target;   
    private final TrajectoryCalculations trajectoryCalculations;

    public Shoot(IndexerSubsystem indexerSubsystem, Supplier<String> fieldLocation, Translation3d target, TrajectoryCalculations trajectoryCalculations) {
        this.m_IndexerSubsystem = indexerSubsystem;
        this.fieldLocation = fieldLocation;
        this.target = target;
        this.trajectoryCalculations = trajectoryCalculations;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        if (!fieldLocation.get().equals("AllianceZone")) {
            trajectoryCalculations.setTargetSupplier(() -> target);
        }
    }

    @Override
    public void execute() {
        m_IndexerSubsystem.setIndexerVelocity(Constants.Indexer.DEFAULT_INDEXER_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        m_IndexerSubsystem.stopMotors();
        trajectoryCalculations.setTargetSupplier(() -> FieldLayout.ShooterTargets.getAllianceHub());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
