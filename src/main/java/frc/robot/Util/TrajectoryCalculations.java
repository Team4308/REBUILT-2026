package frc.robot.Util;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import ca.team4308.absolutelib.math.trajectories.ShotInput;
import ca.team4308.absolutelib.math.trajectories.TrajectorySolver;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig.WheelArrangement;
import ca.team4308.absolutelib.math.trajectories.flywheel.WheelMaterial;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePiece;
import ca.team4308.absolutelib.math.trajectories.gamepiece.GamePieces;
import ca.team4308.absolutelib.math.trajectories.motor.FRCMotors;
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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.FieldLayout;

public class TrajectoryCalculations {
    private final ShooterSystem shooterSystem;
    private final TrajectorySolver trajectorySolver;

    private ShotParameters currentShot = ShotParameters.invalid("No Shot");
    private double targetYawDegrees = 0;
    private double lastDistanceMeters = 0;
    private double lastSolveTimestamp = 0;
    private double lastSolvedDistance = 0;
    private double lastSolvedYawDeg = 0;

    private Supplier<Pose2d> poseSupplier = null;
    private Supplier<ChassisSpeeds> chassisSupplier = null;
    private Supplier<Double> currentRPMsupply = null;
    private final double shooterHeightMeters = Constants.Shooting.TrajectoryCalc.SHOOTER_HEIGHT_M;
    private final Translation2d shooterOffset = new Translation2d(
            Constants.Shooting.TrajectoryCalc.SHOOTER_OFFSET_X_M,
            Constants.Shooting.TrajectoryCalc.SHOOTER_OFFSET_Y_M);
    private Supplier<Translation3d> targetSupplier = () -> FieldLayout.ShooterTargets.kHUB_POSE;

    
    // Setters
    public void setPoseSupplier(Supplier<Pose2d> supplier) {
        this.poseSupplier = supplier;
    }

    public void setChassisSupplier(Supplier<ChassisSpeeds> supplier) {
        this.chassisSupplier = supplier;
    }

    public void setCurrentRPMsupply(Supplier<Double> supplier) {
        this.currentRPMsupply = supplier;
    }

    public boolean suppliersAreSet() {
        return poseSupplier != null && chassisSupplier != null && currentRPMsupply != null;
    }

    public void setTargetSupplier(Supplier<Translation3d> supplier) {
        this.targetSupplier = supplier;
    }

    public TrajectoryCalculations() {
        super();
        TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.defaults()
                .toBuilder()
                .minPitchDegrees(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE)
                .maxPitchDegrees(Constants.Shooting.Hood.FORWARD_SOFT_LIMIT_ANGLE)
                .build();

        GamePiece gamePiece = GamePieces.REBUILT_2026_BALL;

        ShotLookupTable table = new ShotLookupTable();
 
        ShooterConfig shooterConfig = ShooterConfig.builder()
                .pitchLimits(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE,
                        Constants.Shooting.Hood.FORWARD_SOFT_LIMIT_ANGLE)
                .rpmLimits(Constants.Shooting.TrajectoryCalc.MIN_RPM,
                        Constants.Shooting.TrajectoryCalc.MAX_RPM)
                .rpmToVelocityFactor(Constants.Shooting.TrajectoryCalc.RPM_TO_VELOCITY_FACTOR)
                .distanceLimits(Constants.Shooting.TrajectoryCalc.MIN_DISTANCE_M,
                        Constants.Shooting.TrajectoryCalc.MAX_DISTANCE_M)
                .rpmFeedbackThreshold(Constants.Shooting.TrajectoryCalc.RPM_FEEDBACK_THRESHOLD)
                .rpmAbortThreshold(Constants.Shooting.TrajectoryCalc.RPM_ABORT_THRESHOLD)
                .pitchCorrectionPerRpmDeficit(Constants.Shooting.TrajectoryCalc.PITCH_CORRECTION_PER_RPM_DEFICIT)
                .movingCompensationGain(Constants.Shooting.TrajectoryCalc.MOVING_COMPENSATION_GAIN)
                .movingIterations(Constants.Shooting.TrajectoryCalc.MOVING_ITERATIONS)
                .safetyMaxExitVelocity(Constants.Shooting.TrajectoryCalc.SAFETY_MAX_EXIT_VELOCITY)
                .build();
        FlywheelConfig flywheelConfig = FlywheelConfig.builder()
                .arrangement(WheelArrangement.DUAL_OVER_UNDER)
                .wheelDiameterInches(Constants.Shooting.TrajectoryCalc.FLYWHEEL_DIAMETER_IN)
                .material(WheelMaterial.HARD)
                .compressionRatio(Constants.Shooting.TrajectoryCalc.FLYWHEEL_COMPRESSION_RATIO)
                .motor(FRCMotors.KRAKEN_X60)
                .motorsPerWheel(Constants.Shooting.TrajectoryCalc.FLYWHEEL_MOTORS_PER_WHEEL)
                .gearRatio(Constants.Shooting.TrajectoryCalc.FLYWHEEL_GEAR_RATIO)
                .build();

        trajectorySolver = new TrajectorySolver(gamePiece, solverConfig);
        trajectorySolver.setFlywheel(flywheelConfig);
        trajectorySolver.setDebugEnabled(false); // Enable if you need more details about breakdowns of the solution
        trajectorySolver.setSolveMode(TrajectorySolver.SolveMode.CONSTRAINT);

        shooterSystem = new ShooterSystem(shooterConfig, table, trajectorySolver);
        shooterSystem.setMode(ShotMode.SOLVER_ONLY);


    }

    public double getNeededYaw() {
        var result = shooterSystem.getLastTrajectoryResult();
        if (result == null) {
            return 0.0;
        }
        return result.getYawAdjustmentDegrees();
    }

    public double getNeededPitch() {
        var result = shooterSystem.getLastTrajectoryResult();
        if (result == null) {
            return 0.0;
        }
        return result.getPitchAngleDegrees();
    }

    public double getNeededRPM(){
        var result = shooterSystem.getLastTrajectoryResult();
        if (result == null) {
            return 0.0;
        }
        return result.getRecommendedRpm();
    }

    public double getTargetYawDegrees() {
        return targetYawDegrees;
    }

    public double getLastDistanceMeters() {
        return lastDistanceMeters;
    }

    public boolean hasValidShot() {
        return currentShot.valid;
    }

    public ShotParameters getCurrentShot() {
        return currentShot;
    }

    public void updateShot() {
        if (!suppliersAreSet()) {
            Logger.recordOutput("TrajectoryCalc/Error", "Suppliers not set");
            return;
        }

        Pose2d pose = poseSupplier.get();
        Rotation2d rot = pose.getRotation();
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;

        Translation3d target = targetSupplier.get();
        double dx = target.getX() - shooterX;
        double dy = target.getY() - shooterY;
        double yawRad = Math.atan2(dy, dx);
        lastDistanceMeters = Math.hypot(dx, dy);
        targetYawDegrees = Math.toDegrees(yawRad);

        double nowMs = Timer.getFPGATimestamp() * 1000.0;
        double distanceChange = Math.abs(lastDistanceMeters - lastSolvedDistance);
        double yawChange = Math.abs(targetYawDegrees - lastSolvedYawDeg);
        boolean enoughTimePassed = (nowMs - lastSolveTimestamp) >= Constants.Shooting.TrajectoryCalc.MIN_SOLVE_INTERVAL_MS;
        boolean inputsChanged = distanceChange > Constants.Shooting.TrajectoryCalc.DISTANCE_CHANGE_THRESHOLD_M
                || yawChange > Constants.Shooting.TrajectoryCalc.YAW_CHANGE_THRESHOLD_DEG;

        if (!enoughTimePassed && !inputsChanged && currentShot.valid) {
            Logger.recordOutput("TrajectoryCalc/Skipped", true);
            return;
        }

        double vx = 0, vy = 0;

        if (chassisSupplier != null) {
            ChassisSpeeds speeds = chassisSupplier.get();
            vx = speeds.vxMetersPerSecond;
            vy = speeds.vyMetersPerSecond;
        }

        double measuredRPM = currentRPMsupply != null ? currentRPMsupply.get() : 0;
        shooterSystem.setSolverInput(
                ShotInput.builder()
                        .shooterPositionMeters(shooterX, shooterY, shooterHeightMeters)
                        .shooterYawRadians(yawRad)
                        .targetPositionMeters(target.getX(), target.getY(),
                                target.getZ())
                        .targetRadiusMeters(Constants.Shooting.TrajectoryCalc.TARGET_RADIUS_M)
                        .includeAirResistance(true)
                        .robotVelocity(vx, vy)
                        .fastMode()
                        .build());

        currentShot = shooterSystem.calculate(lastDistanceMeters, measuredRPM, vx, vy, yawRad);
        lastSolveTimestamp = nowMs;
        lastSolvedDistance = lastDistanceMeters;
        lastSolvedYawDeg = targetYawDegrees;

        Logger.recordOutput("TrajectoryCalc/Skipped", false);
        Logger.recordOutput("TrajectoryCalc/Valid", currentShot.valid);
        Logger.recordOutput("TrajectoryCalc/Distance", lastDistanceMeters);
        Logger.recordOutput("TrajectoryCalc/YawDeg", targetYawDegrees);
        Logger.recordOutput("TrajectoryCalc/Pitch", currentShot.pitchDegrees);
        Logger.recordOutput("TrajectoryCalc/RPM", currentShot.rpm);
        Logger.recordOutput("TrajectoryCalc/Source", currentShot.source.name());
    }

}