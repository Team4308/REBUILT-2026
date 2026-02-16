package frc.robot.commands;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

import ca.team4308.absolutelib.math.trajectories.ShotInput;
import ca.team4308.absolutelib.math.trajectories.TrajectorySolver;
import ca.team4308.absolutelib.math.trajectories.physics.ProjectileMotion.TrajectoryResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.HoodedShooter;

/**
 * Command that queries the TrajectorySolver for a shot given the robot/shooter
 * pose, moves the hood to the solver's angle, and publishes the desired
 * flywheel RPM. This implementation uses reflection to call the solver's
 * solve method and to read the returned solution so it does not depend on the
 * concrete solution class at compile time.
 */
public class HoodedPivot extends Command {
    private final HoodedShooter m_hood;
    private final Supplier<Pose2d> m_poseSupplier;
    private boolean m_finished = false;

    public HoodedPivot(HoodedShooter hood, Supplier<Pose2d> poseSupplier) {
        this.m_hood = hood;
        this.m_poseSupplier = poseSupplier;

        addRequirements(hood.getPivot());
    }

    @Override
    public void initialize() {
        try {
            TrajectorySolver solver = TrajectorySolver.forGame2026();

            Pose2d pose = m_poseSupplier.get();
            double shooterZ = 0.5;

            ShotInput input = m_hood.SloveInput(pose.getX(), pose.getY(), shooterZ);

            Method solveMethod = Arrays.stream(solver.getClass().getMethods())
                    .filter(m -> m.getParameterCount() == 1 && m.getParameterTypes()[0].equals(ShotInput.class))
                    .findFirst()
                    .orElse(null);

            if (solveMethod == null) {
                SmartDashboard.putString("HoodedPivot/Error", "No solver method accepting ShotInput found");
                m_finished = true;
                return;
            }

            ca.team4308.absolutelib.math.trajectories.TrajectoryResult result = solver.solve(input);


            SmartDashboard.putNumber("Shooter/DesiredHoodAngle", result.getPitchAngleDegrees());
            SmartDashboard.putNumber("Shooter/Conf", result.getConfidenceScore());
            SmartDashboard.putNumber("Shooter/DesiredRPM", result.getDiscreteRpm());
            m_hood.getPivot().setPosition(result.getPitchAngleDegrees());
        } catch (Exception e) {
            SmartDashboard.putString("HoodedPivot/Exception", e.toString());
            e.printStackTrace();
        } finally {
            m_finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
