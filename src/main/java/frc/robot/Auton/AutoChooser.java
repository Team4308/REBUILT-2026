package frc.robot.Auton;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Auton.Routines.IdleAuto;
import frc.robot.Auton.Routines.MiddleDepotAuto;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.IndexerSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class AutoChooser extends SendableChooser<Auto> {

    private final Map<Auto, AutoProgram> autoPrograms = new HashMap<>();

    public AutoChooser(SwerveSubsystem drivebase, HoodSubsystem m_HoodSubsystem, IntakeSubsystem m_IntakeSubsystem,
            TurretSubsystem m_TurretSubsystem,
            IndexerSubsystem m_IndexerSubsystem, ShooterSubsystem m_ShooterSubsystem) {
        List<AutoProgram> programs = List.of(
                new IdleAuto(drivebase),
                new MiddleDepotAuto(drivebase, m_IndexerSubsystem, m_IntakeSubsystem));

        for (AutoProgram program : programs) {
            autoPrograms.put(program.getAuto(), program);

            if (program.getAuto() == Auto.IDLE) {
                setDefaultOption(program.getLabel(), program.getAuto());
            } else {
                addOption(program.getLabel(), program.getAuto());
            }
        }
    }

    /**
     * Returns the {@link AutoProgram} (a schedulable Command) for the currently
     * selected auto. Falls back to IDLE if nothing is selected or the selection
     * is unrecognised.
     */
    public AutoProgram getSelectedAutoProgram() {
        Auto selected = getSelected();
        return autoPrograms.getOrDefault(selected, autoPrograms.get(Auto.IDLE));
    }

    /**
     * Convenience getter — returns the starting pose of the selected auto so
     * RobotContainer can seed odometry before the program is scheduled.
     */
    public Pose2d getSelectedStartingPose() {
        return getSelectedAutoProgram().getStartingPose();
    }
}