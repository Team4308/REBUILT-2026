package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldLayout;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

/**
 * A {@link SequentialCommandGroup} that resets the robot's odometry to a
 * given starting pose before running its commands.
 *
 * <p>
 * Usage:
 * 
 * <pre>
 * new AutoFactory(drive, new Pose2d(1.5, 4.0, new Rotation2d()),
 *         new DriveForwardCommand(drive),
 *         new TurnCommand(drive));
 * </pre>
 */
public class AutoProgram extends SequentialCommandGroup {

    private final Pose2d startingPose;
    private final String label;
    private final Auto auto;

    /**
     * Creates a new AutoFactory.
     *
     * @param drive        The drive subsystem used to reset odometry.
     * @param label        The label for this auto program.
     * @param startingPose The pose the robot should start at when this auto begins.
     * @param commands     The commands to run sequentially after the odometry
     *                     reset.
     */
    public AutoProgram(Auto auto, String label, SwerveSubsystem drivebase, Pose2d startingPose, Command... commands) {
        this.startingPose = startingPose;
        this.label = label;
        this.auto = auto;

        // Prepend an odometry reset before the provided commands
        addCommands(
                new InstantCommand(() -> drivebase.resetOdometry(startingPose)).withName("Resetting Odometry"));
        addCommands(commands);
    }

    /**
     * Returns the starting pose this auto was constructed with.
     *
     * @return The starting {@link Pose2d}.
     */
    public Pose2d getStartingPose() {
        return startingPose;
    }

    public Auto getAuto() {
        return auto;
    }

    public String getLabel() {
        return label;
    }

    protected Pose2d getAllianceFlippedPose2d(Pose2d pose) {
        if (startingPose.getX() > FieldLayout.kFieldLength / 2.0) {
            return new Pose2d(FieldLayout.kFieldLength - pose.getX(), pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            return pose;
        }
    }
}