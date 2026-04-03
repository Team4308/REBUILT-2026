package frc.robot.Auton.Routines;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Auton.Auto;
import frc.robot.Auton.AutoProgram;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;

public class IdleAuto extends AutoProgram {
    public IdleAuto(SwerveSubsystem drivebase) {
        super(Auto.IDLE, "Idle", drivebase, new Pose2d());
    }
}