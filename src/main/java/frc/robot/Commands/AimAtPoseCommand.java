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
import frc.robot.Util.AllianceFlipUtil;

public class AimAtPoseCommand extends Command {
    private Supplier<Pose2d> botPose;
    private Translation2d shooterOffset = new Translation2d(0.13, 0.0);
    private Translation3d targetPose;
    private TurretSubsystem turretSubsystem;

    public AimAtPoseCommand(Supplier<Pose2d> botPosetPose, TurretSubsystem turretSubsystem,
            Translation3d targetPose) {
        botPose = botPosetPose;
        this.turretSubsystem = turretSubsystem;
        this.targetPose = targetPose;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        targetPose = AllianceFlipUtil.apply(targetPose);
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
        double dx = targetPose.getX() - shooterX;
        double dy = targetPose.getY() - shooterY;
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngleDeg = ((Rotation2d.fromDegrees(fieldAngleDeg).minus(rot).getDegrees()) % 360 + 360)
                % 360;
        turretSubsystem.setTarget(turretAngleDeg - 180);

        Logger.recordOutput("Commands/AimAtPose/Robot/Rot", rot.getDegrees());
        Logger.recordOutput("Commands/AimAtPose/Robot/X", shooterX);
        Logger.recordOutput("Commands/AimAtPose/Robot/Y", shooterY);
        Logger.recordOutput("Commands/AimAtPose/Target/FieldDeg", fieldAngleDeg);
        Logger.recordOutput("Commands/AimAtPose/Target/TurretDeg", turretAngleDeg);
        Logger.recordOutput("Commands/AimAtPose/Target/dX", dx);
        Logger.recordOutput("Commands/AimAtPose/Robot/dY", dy);
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