package frc.robot.Commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.TurretSubsystem;

public class AimAtHubCommand extends CommandBase {
    private Supplier<Pose2d> botPose;
    private Translation2d shooterOffset = new Translation2d(0.1, 0.1);
    private Translation3d hubTransalation = new Translation3d(4.0, 0.0, 2.1); // Blue hub
    private TurretSubsystem turretSubsystem;

    public AimAtHubCommand(Supplier<Pose2d> botPosetPose, TurretSubsystem turretSubsystem) {
        botPose = botPosetPose;
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            hubTransalation = new Translation3d(4.0, 0.0, 9.9);
        }
    }

    @Override
    public void execute() {
        Pose2d pose = new Pose2d(botPose.get().getX(), botPose.get().getY(),
                botPose.get().getRotation());
        Rotation2d rot = pose.getRotation();
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;
        double dx = hubTransalation.getX() - shooterX;
        double dy = hubTransalation.getY() - shooterY;
        double yawRad = Math.atan2(dy, dx);
        turretSubsystem.setTarget(Math.toDegrees(yawRad));

        Logger.recordOutput("Commands/AimAtHub/Robot/Rot", rot.getDegrees());
        Logger.recordOutput("Commands/AimAtHub/Robot/X", shooterX);
        Logger.recordOutput("Commands/AimAtHub/Robot/Y", shooterY);
        Logger.recordOutput("Commands/AimAtHub/Target/Deg", Math.toDegrees(yawRad));
        Logger.recordOutput("Commands/AimAtHub/Target/dX", dx);
        Logger.recordOutput("Commands/AimAtHub/Robot/dY", dy);

    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}