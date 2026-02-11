package frc.robot.Util;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import ca.team4308.absolutelib.math.trajectories.ShotInput;
import ca.team4308.absolutelib.math.trajectories.TrajectorySolver;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces;
import ca.team4308.absolutelib.math.trajectories.shooter.ShooterConfig;
import ca.team4308.absolutelib.math.trajectories.shooter.ShooterSystem;
import ca.team4308.absolutelib.math.trajectories.shooter.ShotLookupTable;
import ca.team4308.absolutelib.math.trajectories.shooter.ShotMode;
import ca.team4308.absolutelib.math.trajectories.shooter.ShotParameters;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

public class trajCalc {
    private final ShooterSystem shooterSystem;
    private final TrajectorySolver trajectorySolver;

    private ShotParameters currentShot = ShotParameters.invalid("No Shot");
    private double targetYawDegrees = 0;
    private double lastDistantMeter = 0;

    private Supplier<Pose2d> poseSupplier = null;
    private Supplier<ChassisSpeeds> chassisSupplier = null;
    private Supplier<Double> currentRPMsupply = null;
    private double shooterHeightMeters = 0.5;
    private Translation2d shooterOffset = new Translation2d(0.1,0.1);
    private Translation3d hub = new Translation3d(4.0, 0.0,2.1);

    public trajCalc() {
        super();
        TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.defaults()
                .toBuilder()
                .minPitchDegrees(Constants.Hood.REVERSE_SOFT_LIMIT_ANGLE)
                .maxPitchDegrees(Constants.Hood.FORWARD_SOFT_LIMIT_ANGLE)
                .build();

        GamePiece gamePiece = GamePieces.REBUILT_2026_BALL;

        ShotLookupTable table = new ShotLookupTable();

        ShooterConfig shooterConfig = ShooterConfig.builder()
                .pitchLimits(Constants.Hood.REVERSE_SOFT_LIMIT_ANGLE, Constants.Hood.FORWARD_SOFT_LIMIT_ANGLE)
                .rpmLimits(0.0, 6000.0)
                .rpmToVelocityFactor(0.00532)
                .distanceLimits(0.5, 12.0)
                .rpmFeedbackThreshold(50.0)
                .rpmAbortThreshold(500.0)
                .pitchCorrectionPerRpmDeficit(0.05)
                .movingCompensationGain(1)
                .movingIterations(5)
                .safetyMaxExitVelocity(30)
                .build();
        trajectorySolver = new TrajectorySolver(gamePiece, solverConfig);
        trajectorySolver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

        shooterSystem = new ShooterSystem(shooterConfig,table,trajectorySolver);
        shooterSystem.setMode(ShotMode.SOLVER_WITH_LOOKUP_FALLBACK);
        
    }

    public double getNeededYaw() {
        return shooterSystem.getLastTrajectoryResult().getYawAdjustmentDegrees();
    }

    public double getNeededPitch() {
        return shooterSystem.getLastTrajectoryResult().getPitchAngleDegrees();
    }

    public double getNeededRPM(){
        return shooterSystem.getLastTrajectoryResult().getRecommendedRpm();
    }

    public void updateShot(){
        Pose2d pose = poseSupplier.get();
        Rotation2d rot = pose.getRotation();
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getY() * rot.getSin() + shooterOffset.getX() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;
        

        double dx = hub.getX() - shooterX;
        double dy = hub.getY() - shooterY;
        double yawRad = Math.atan2(dx, dy);
        lastDistantMeter = Math.hypot(dx, dy);
        targetYawDegrees = Math.toDegrees(yawRad);
        double vx = 0, vy = 0;

        if(chassisSupplier != null){
            ChassisSpeeds speeds = chassisSupplier.get();
            vx = speeds.vxMetersPerSecond;
            vy = speeds.vyMetersPerSecond;
        }

        double measuredRPM = currentRPMsupply != null ? currentRPMsupply.get() : 0;
        shooterSystem.setSolverInput(
            ShotInput.builder()
                .shooterPositionMeters(shooterX, shooterY, shooterHeightMeters)
                .shooterYawRadians(yawRad)
                .targetPositionMeters(hub.getX(), hub.getY(), hub.getZ())
                .targetRadiusMeters(0.45)
                .includeAirResistance(true)
                .robotVelocity(vx, vy)
                .build()
        );

        currentShot = shooterSystem.calculate(lastDistantMeter, measuredRPM, vx, vy, yawRad);
    }   

    

}