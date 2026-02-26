
package frc.robot.commands;

import frc.robot.subsystems.Climbersubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignRightThenClimbCommands {
    // AutoAlignRightThenClimbCommand // This will retract the intake, move climbers
    // up, then drive to the right/left side, then do the climbing stuff
    // You need to use drivebase.drivetopose() for that.

    // AutoAlignRightThenClimbCommand

    // private final Climbersubsystem m_climb;
    // private final DriveSubsystem m_drive;

    public autoAlignLeftThenClimb (Climbersubsystem climber, DriveSubsystem drive, Pose2d rightPose) 
    {
       return Commands.sequence(
                Commands.runOnce(climber::retractClimb, climber),
                drive.driveToPose(rightPose),
                Commands.runOnce(climber::climb, climber)
        );
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
