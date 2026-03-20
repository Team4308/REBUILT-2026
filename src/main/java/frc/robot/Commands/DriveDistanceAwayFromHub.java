package frc.robot.Commands;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import frc.robot.Util.AllianceFlipUtil;
import frc.robot.Util.FieldConstants;

public class DriveDistanceAwayFromHub extends SequentialCommandGroup {

    public DriveDistanceAwayFromHub(SwerveSubsystem drivebase, Supplier<Double> distanceFromHub) {
        addCommands(Commands.defer(() -> {
            Translation2d hub = AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
            Translation2d robotPos = drivebase.getPose().getTranslation();
            Translation2d hubToRobot = robotPos.minus(hub);
            double currentDist = hubToRobot.getNorm();
            Translation2d targetTranslation = currentDist > 1e-6
                    ? hub.plus(hubToRobot.times(distanceFromHub.get() / currentDist))
                    : hub.plus(new Translation2d(distanceFromHub.get(), 0));
            return drivebase.driveToPoseObjAvoid(
                    () -> new Pose2d(targetTranslation, hub.minus(targetTranslation).getAngle()));
        }, Set.of(drivebase)));
    }

    public DriveDistanceAwayFromHub(SwerveSubsystem drivebase, double distanceFromHub) {
        this(drivebase, () -> distanceFromHub);
    }
}