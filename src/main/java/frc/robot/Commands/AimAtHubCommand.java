package frc.robot.Commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldLayout;
import frc.robot.Subsystems.TurretSubsystem;

public class AimAtHubCommand extends Command {
    private Supplier<Pose2d> botPose;
    private Translation2d shooterOffset = new Translation2d(0.1, 0.1);
    private Translation3d hubTranslation;
    private TurretSubsystem turretSubsystem;

    public AimAtHubCommand(Supplier<Pose2d> botPosetPose, TurretSubsystem turretSubsystem) {
        botPose = botPosetPose;
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        hubTranslation = FieldLayout.ShooterTargets.getAllianceHub();
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
        double dx = hubTranslation.getX() - shooterX;
        double dy = hubTranslation.getY() - shooterY;
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngleDeg = ((Rotation2d.fromDegrees(fieldAngleDeg).minus(rot).getDegrees() + 180.0) % 360 + 360)
                % 360;
        turretSubsystem.setTarget(turretAngleDeg);

        Logger.recordOutput("Commands/AimAtHub/Robot/Rot", rot.getDegrees());
        Logger.recordOutput("Commands/AimAtHub/Robot/X", shooterX);
        Logger.recordOutput("Commands/AimAtHub/Robot/Y", shooterY);
        Logger.recordOutput("Commands/AimAtHub/Target/FieldDeg", fieldAngleDeg);
        Logger.recordOutput("Commands/AimAtHub/Target/TurretDeg", turretAngleDeg);
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