package frc.robot.Commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.Supplier;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Shooting.Turret;
import frc.robot.Subsystems.TurretSubsystem;

public class AimAtHubCommand extends Command {
    private Pose3d botPose;
    private Translation2d shooterOffset = new Translation2d(0.0, 0.0);
    private Supplier<Translation3d> targetSupplier = () -> new Translation3d(4.0, 0.0, 2.1); // Blue hub
    private TurretSubsystem turretSubsystem;
    public AimAtHubCommand(Pose3d botPosetPose, TurretSubsystem turretSubsystem) {
        botPose = botPosetPose;
        this.turretSubsystem = turretSubsystem;
    }

    
    @Override
    public void execute() {
        Pose2d pose = new Pose2d(botPose.getX(), botPose.getY(), botPose.getRotation().toRotation2d());
        Rotation2d rot = pose.getRotation();
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;
        double dx = targetSupplier.get().getX() - shooterX;
        double dy = targetSupplier.get().getY() - shooterY;
        double yawRad = Math.atan2(dy, dx);
        turretSubsystem.setTarget(Math.toDegrees(yawRad));
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
    @Override
    public void initialize() {
        turretSubsystem.stopMotors();
    }

}
