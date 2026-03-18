package frc.robot.Commands.Trajectories;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Util.LaunchCalculator;
import frc.robot.Subsystems.IndexerSubsystem;

public class Shoot extends Command{
    private final IndexerSubsystem m_IndexerSubsystem;
    private final Supplier<String> fieldLocation;
    private final Supplier<LaunchCalculator.LaunchingParameters> launchParamsSupplier;

    public Shoot(
        IndexerSubsystem indexerSubsystem,
        Supplier<String> fieldLocation,
        Supplier<LaunchCalculator.LaunchingParameters> launchParamsSupplier) {
        this.m_IndexerSubsystem = indexerSubsystem;
        this.fieldLocation = fieldLocation;
        this.launchParamsSupplier = launchParamsSupplier;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void initialize() {
        if (!fieldLocation.get().equals("AllianceZone")) {
        }
        LaunchCalculator.LaunchingParameters params = launchParamsSupplier.get();
        if (params != null) {
            Logger.recordOutput("Commands/Shoot/Target", params.target());
            Logger.recordOutput("Commands/Shoot/Passing", params.passing());
            Logger.recordOutput("Commands/Shoot/PassingLeft", params.passingLeft());
        }
    }

    @Override
    public void execute() {
        LaunchCalculator.LaunchingParameters params = launchParamsSupplier.get();

        if (params != null) {
            Logger.recordOutput("Commands/Shoot/Target", params.target());
            Logger.recordOutput("Commands/Shoot/Passing", params.passing());
        }

        // Only run the indexer / hopper if we have a valid launch solution.
        if (params != null && params.isValid()) {
            m_IndexerSubsystem.setHopperVelocity(Constants.Indexer.DEFAULT_HOPPER_VELOCITY);
            m_IndexerSubsystem.setIndexerVelocity(Constants.Indexer.DEFAULT_INDEXER_VELOCITY);
        } else {
            m_IndexerSubsystem.stopMotors();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_IndexerSubsystem.stopMotors();

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
