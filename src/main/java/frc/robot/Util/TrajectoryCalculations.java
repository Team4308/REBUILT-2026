package frc.robot.Util;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import ca.team4308.absolutelib.math.trajectories.ShotInput;
import ca.team4308.absolutelib.math.trajectories.SolveDebugInfo;
import ca.team4308.absolutelib.math.trajectories.SolverConstants;
import ca.team4308.absolutelib.math.trajectories.TrajectoryResult;
import ca.team4308.absolutelib.math.trajectories.TrajectorySolver;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelConfig.WheelArrangement;
import ca.team4308.absolutelib.math.trajectories.flywheel.FlywheelSimulator;
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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.FieldLayout;

public class TrajectoryCalculations {
    private final ShooterSystem shooterSystem;
    private final TrajectorySolver trajectorySolver;

    private ShotParameters currentShot = ShotParameters.invalid("Not yet calculated");
    private double targetYawDegrees = 0.0;
    private double lastDistanceMeters = 0.0;
    private double lastComputationTimeMs = 0.0;

    private double lastSolveTimestamp = 0;
    private double lastSolvedDistance = 0;
    private double lastSolvedYawDeg = 0;

    private Supplier<Pose2d> poseSupplier = null;
    private Supplier<ChassisSpeeds> chassisSupplier = null;
    private Supplier<Double> currentRPMsupply = null;

    private double shooterHeightMeters = Constants.Shooting.TrajectoryCalc.SHOOTER_HEIGHT_M;
    private Translation2d shooterOffset = new Translation2d(
            Constants.Shooting.TrajectoryCalc.SHOOTER_OFFSET_X_M,
            Constants.Shooting.TrajectoryCalc.SHOOTER_OFFSET_Y_M);

    private Translation3d targetPosition = FieldLayout.ShooterTargets.kHUB_POSE;
    private Supplier<Translation3d> targetSupplier = null;

    private boolean trackingEnabled = true;
    private boolean loggingEnabled = true;

    public TrajectoryCalculations() {
        super();

        SolverConstants.setMinTargetDistanceMeters(0.05);
        SolverConstants.setVelocityBufferMultiplier(1.2);
        SolverConstants.setRimClearanceMeters(0.05);
        SolverConstants.setMinEntryAngleDegrees(45.0);
        SolverConstants.setDragCompensationMultiplier(1.5);

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
                .name("REBUILT-2026 Shooter")
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
        trajectorySolver.setDebugEnabled(true);
        trajectorySolver.setSolveMode(TrajectorySolver.SolveMode.SWEEP);

        shooterSystem = new ShooterSystem(shooterConfig, table, trajectorySolver);
        shooterSystem.setMode(ShotMode.SOLVER_ONLY);
        shooterSystem.setFallbackShot(60.0, 3000);
    }

    public void setPoseSupplier(Supplier<Pose2d> supplier) {
        this.poseSupplier = supplier;
    }

    public void setChassisSupplier(Supplier<ChassisSpeeds> supplier) {
        this.chassisSupplier = supplier;
    }

    public void setCurrentRPMsupply(Supplier<Double> supplier) {
        this.currentRPMsupply = supplier;
    }

    public void setTargetSupplier(Supplier<Translation3d> supplier) {
        this.targetSupplier = supplier;
    }

    public void setTarget(double x, double y, double z) {
        this.targetPosition = new Translation3d(x, y, z);
    }

    public void setShooterHeight(double meters) {
        this.shooterHeightMeters = meters;
    }

    public void setShooterOffset(Translation2d offset) {
        this.shooterOffset = offset;
    }

    public void setTrackingEnabled(boolean enabled) {
        this.trackingEnabled = enabled;
    }

    public void setLoggingEnabled(boolean enabled) {
        this.loggingEnabled = enabled;
    }

    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    public void setMode(ShotMode mode) {
        shooterSystem.setMode(mode);
    }

    public ShotMode getMode() {
        return shooterSystem.getMode();
    }

    public void setManualOverride(double pitchDegrees, double rpm) {
        shooterSystem.setManualOverride(pitchDegrees, rpm);
    }

    public double getNeededYaw() {
        var result = shooterSystem.getLastTrajectoryResult();
        return result != null ? result.getYawAdjustmentDegrees() : 0.0;
    }

    public double getNeededPitch() {
        var result = shooterSystem.getLastTrajectoryResult();
        return result != null ? result.getPitchAngleDegrees() : 0.0;
    }

    public double getNeededRPM() {
        var result = shooterSystem.getLastTrajectoryResult();
        return result != null ? result.getRecommendedRpm() : 0.0;
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

    public ShooterSystem getShooterSystem() {
        return shooterSystem;
    }

    public double getLastComputationTimeMs() {
        return lastComputationTimeMs;
    }

    public boolean isReadyToFire() {
        double rpm = currentRPMsupply != null ? currentRPMsupply.get() : 0;
        return shooterSystem.isReadyToFire(rpm);
    }

    public boolean suppliersAreSet() {
        return poseSupplier != null && chassisSupplier != null && currentRPMsupply != null;
    }

    public void updateShot() {
        if (!suppliersAreSet()) {
            Logger.recordOutput("TrajectoryCalc/Error", "Suppliers not set");
            return;
        }

        if (targetSupplier != null) {
            targetPosition = targetSupplier.get();
        }

        Pose2d pose = poseSupplier.get();
        Rotation2d rot = pose.getRotation();
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;

        double dx = targetPosition.getX() - shooterX;
        double dy = targetPosition.getY() - shooterY;
        double yawRad = Math.atan2(dy, dx);
        lastDistanceMeters = Math.hypot(dx, dy);
        targetYawDegrees = Math.toDegrees(yawRad);

        double nowMs = Timer.getFPGATimestamp() * 1000.0;
        double distanceChange = Math.abs(lastDistanceMeters - lastSolvedDistance);
        double yawChange = Math.abs(targetYawDegrees - lastSolvedYawDeg);
        boolean enoughTimePassed = (nowMs
                - lastSolveTimestamp) >= Constants.Shooting.TrajectoryCalc.MIN_SOLVE_INTERVAL_MS;
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
                        .targetPositionMeters(targetPosition.getX(), targetPosition.getY(),
                                targetPosition.getZ())
                        .targetRadiusMeters(Constants.Shooting.TrajectoryCalc.TARGET_RADIUS_M)
                        .includeAirResistance(true)
                        .robotVelocity(vx, vy)
                        .build());

        long startTime = System.nanoTime();
        currentShot = shooterSystem.calculate(lastDistanceMeters, measuredRPM, vx, vy, yawRad);
        long endTime = System.nanoTime();
        lastComputationTimeMs = (endTime - startTime) / 1_000_000.0;

        lastSolveTimestamp = nowMs;
        lastSolvedDistance = lastDistanceMeters;
        lastSolvedYawDeg = targetYawDegrees;

        if (loggingEnabled) {
            Logger.recordOutput("TrajectoryCalc/Skipped", false);
            Logger.recordOutput("TrajectoryCalc/RobotX", pose.getX());
            Logger.recordOutput("TrajectoryCalc/RobotY", pose.getY());
            Logger.recordOutput("TrajectoryCalc/ShooterX", shooterX);
            Logger.recordOutput("TrajectoryCalc/ShooterY", shooterY);
            Logger.recordOutput("TrajectoryCalc/RPM Required", currentShot.rpm);
            Logger.recordOutput("TrajectoryCalc/Pitch Required", currentShot.pitchDegrees);
            Logger.recordOutput("TrajectoryCalc/Yaw Required", targetYawDegrees);
            Logger.recordOutput("TrajectoryCalc/Distance", lastDistanceMeters);
            Logger.recordOutput("TrajectoryCalc/ComputationTimeMs", lastComputationTimeMs);

        }
    }

    public void calculateShot(double shooterX, double shooterY, double shooterZ,
            double targetX, double targetY, double targetZ) {
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;
        lastDistanceMeters = Math.hypot(dx, dy);
        targetYawDegrees = Math.toDegrees(Math.atan2(dy, dx));
        currentShot = shooterSystem.calculate(lastDistanceMeters);
    }

    public void cycleMode() {
        ShotMode[] modes = ShotMode.values();
        int next = (shooterSystem.getMode().ordinal() + 1) % modes.length;
        shooterSystem.setMode(modes[next]);
        System.out.println("Shot mode: " + modes[next]);
    }

    public void shootBallSim() {
        if (poseSupplier == null) {
            return;
        }

        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds speeds = chassisSupplier != null ? chassisSupplier.get() : new ChassisSpeeds();
        TrajectoryResult trajResult = shooterSystem.getLastTrajectoryResult();
        if (trajResult == null || !trajResult.isSuccess()) {
            System.out.println("Cannot shoot: no valid trajectory");
            return;
        }

        double launchSpeed = trajResult.getRequiredVelocityMps();
        double pitchRad = Math.toRadians(trajResult.getPitchAngleDegrees());
        double yawRad = Math.toRadians(targetYawDegrees) + trajResult.getYawAdjustmentRadians();

        Rotation2d rot = robotPose.getRotation();
        double wx = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double wy = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        Translation3d pos = new Translation3d(robotPose.getX() + wx, robotPose.getY() + wy, shooterHeightMeters);

        double hSpeed = launchSpeed * Math.cos(pitchRad);
        Translation3d vel = new Translation3d(
                hSpeed * Math.cos(yawRad) + speeds.vxMetersPerSecond,
                hSpeed * Math.sin(yawRad) + speeds.vyMetersPerSecond,
                launchSpeed * Math.sin(pitchRad));

        FuelSim.getInstance().spawnFuel(pos, vel);
    }

    // Do not log this every cycle - only when a new shot is calculated or when
    // significant changes occur
    private void logShotOutput() {
        if (!loggingEnabled) {
            return;
        }

        Logger.recordOutput("TrajectoryCalc/ValidShot", currentShot.valid);
        Logger.recordOutput("TrajectoryCalc/TargetRPM", currentShot.rpm);
        Logger.recordOutput("TrajectoryCalc/TargetPitchDeg", currentShot.pitchDegrees);
        Logger.recordOutput("TrajectoryCalc/TargetYawDeg", targetYawDegrees);
        Logger.recordOutput("TrajectoryCalc/Distance", lastDistanceMeters);
        Logger.recordOutput("TrajectoryCalc/ShotSource", currentShot.source.name());
        Logger.recordOutput("TrajectoryCalc/Mode", shooterSystem.getMode().name());
        Logger.recordOutput("TrajectoryCalc/SourceDetail", shooterSystem.getLastSourceDescription());
        Logger.recordOutput("TrajectoryCalc/TrackingEnabled", trackingEnabled);
        Logger.recordOutput("TrajectoryCalc/ExitVelocity", currentShot.exitVelocityMps);
        Logger.recordOutput("TrajectoryCalc/ComputationTimeMs", lastComputationTimeMs);

        if (currentRPMsupply != null) {
            double measured = currentRPMsupply.get();
            Logger.recordOutput("TrajectoryCalc/MeasuredRPM", measured);
            Logger.recordOutput("TrajectoryCalc/RpmDeficit", currentShot.rpm - measured);
            Logger.recordOutput("TrajectoryCalc/ReadyToFire", shooterSystem.isReadyToFire(measured));
        }

        Pose3d goalPose = new Pose3d(targetPosition, new Rotation3d());
        Logger.recordOutput("TrajectoryCalc/GoalPose3d", goalPose);
        Logger.recordOutput("TrajectoryCalc/GoalPose3dArray", new Pose3d[] { goalPose });

        Pose3d shooterYawPose = new Pose3d(
                new Translation3d(0, 0, 0),
                new Rotation3d(0, 0, Math.toRadians(targetYawDegrees)));
        Logger.recordOutput("TrajectoryCalc/ShooterYawPose3d", shooterYawPose);

        Logger.recordOutput("TrajectoryCalc/TargetX", targetPosition.getX());
        Logger.recordOutput("TrajectoryCalc/TargetY", targetPosition.getY());
        Logger.recordOutput("TrajectoryCalc/TargetZ", targetPosition.getZ());
        Logger.recordOutput("TrajectoryCalc/ShooterHeight", shooterHeightMeters);
    }

    private void logTrajectoryDebug() {
        if (!loggingEnabled) {
            return;
        }

        TrajectoryResult trajResult = shooterSystem.getLastTrajectoryResult();
        if (trajResult == null) {
            return;
        }

        Logger.recordOutput("Trajectory/Status", trajResult.getStatus().name());
        Logger.recordOutput("Trajectory/StatusMessage", trajResult.getStatusMessage());
        Logger.recordOutput("Trajectory/Confidence", trajResult.getConfidenceScore());
        Logger.recordOutput("Trajectory/ComputationTimeMs", lastComputationTimeMs);

        if (trajResult.isSuccess()) {
            Logger.recordOutput("Trajectory/PitchDeg", trajResult.getPitchAngleDegrees());
            Logger.recordOutput("Trajectory/YawAdjustDeg", trajResult.getYawAdjustmentDegrees());
            Logger.recordOutput("Trajectory/Velocity", trajResult.getRequiredVelocityMps());
            Logger.recordOutput("Trajectory/TimeOfFlight", trajResult.getTimeOfFlightSeconds());
            Logger.recordOutput("Trajectory/MaxHeight", trajResult.getMaxHeightMeters());
            Logger.recordOutput("Trajectory/Margin", trajResult.getMarginOfErrorMeters());
            Logger.recordOutput("Trajectory/RPM", trajResult.getRecommendedRpm());

            FlywheelSimulator.SimulationResult flywheelSim = trajResult.getFlywheelSimulation();
            if (flywheelSim != null) {
                Logger.recordOutput("Flywheel/ExitVelocityMps", flywheelSim.exitVelocityMps);
                Logger.recordOutput("Flywheel/MotorPowerPercent", flywheelSim.motorPowerPercent);
                Logger.recordOutput("Flywheel/RequiredMotorRpm", flywheelSim.requiredMotorRpm);
                Logger.recordOutput("Flywheel/RequiredWheelRpm", flywheelSim.requiredWheelRpm);
                Logger.recordOutput("Flywheel/SpinUpTimeSeconds", flywheelSim.spinUpTimeSeconds);
                Logger.recordOutput("Flywheel/CurrentDrawAmps", flywheelSim.currentDrawAmps);
                Logger.recordOutput("Flywheel/StoredEnergyJoules", flywheelSim.storedEnergyJoules);
                Logger.recordOutput("Flywheel/BallSpinRpm", flywheelSim.ballSpinRpm);
                Logger.recordOutput("Flywheel/ContactTimeMs", flywheelSim.contactTimeMs);
                Logger.recordOutput("Flywheel/EnergyEfficiency", flywheelSim.energyTransferEfficiency);
                Logger.recordOutput("Flywheel/SlipRatio", flywheelSim.slipRatio);
                Logger.recordOutput("Flywheel/IsAchievable", flywheelSim.isAchievable);
                Logger.recordOutput("Flywheel/LimitingFactor", flywheelSim.limitingFactor);
            }

            List<Pose3d> flightPath = trajResult.getFlightPath();
            if (!flightPath.isEmpty()) {
                Logger.recordOutput("TrajectoryCalc/Trajectory/FlightPath",
                        flightPath.toArray(new Pose3d[0]));

                double[] pathX = new double[flightPath.size()];
                double[] pathY = new double[flightPath.size()];
                double[] pathZ = new double[flightPath.size()];
                for (int i = 0; i < flightPath.size(); i++) {
                    pathX[i] = flightPath.get(i).getX();
                    pathY[i] = flightPath.get(i).getY();
                    pathZ[i] = flightPath.get(i).getZ();
                }
                Logger.recordOutput("Trajectory/PathX", pathX);
                Logger.recordOutput("Trajectory/PathY", pathY);
                Logger.recordOutput("Trajectory/PathZ", pathZ);
                Logger.recordOutput("Trajectory/PathLength", flightPath.size());
            }
        }

        SolveDebugInfo debug = trajResult.getDebugInfo();
        if (debug != null) {
            Logger.recordOutput("Debug/TotalTested", debug.getTotalTested());
            Logger.recordOutput("Debug/Accepted", debug.getAcceptedCount());
            Logger.recordOutput("Debug/TotalRejected", debug.getTotalRejected());
            Logger.recordOutput("Debug/RejectedCollision", debug.getRejectedCollisionCount());
            Logger.recordOutput("Debug/RejectedArcTooLow", debug.getRejectedArcTooLowCount());
            Logger.recordOutput("Debug/RejectedClearance", debug.getRejectedClearanceCount());
            Logger.recordOutput("Debug/RejectedMiss", debug.getRejectedMissCount());
            Logger.recordOutput("Debug/RejectedFlyover", debug.getRejectedFlyoverCount());
            Logger.recordOutput("Debug/BestMissDistance", debug.getBestMissDistance());
            Logger.recordOutput("Debug/BestPitchDeg", debug.getBestPitchDegrees());
            Logger.recordOutput("Debug/Summary", debug.getSummary());
            Logger.recordOutput("Debug/DetailedTable", debug.getDetailedTable());

            List<SolveDebugInfo.CandidateInfo> accepted = debug.getAcceptedCandidates();
            double[] accPitch = new double[accepted.size()];
            double[] accMiss = new double[accepted.size()];
            double[] accTOF = new double[accepted.size()];
            double[] accMaxH = new double[accepted.size()];
            for (int i = 0; i < accepted.size(); i++) {
                accPitch[i] = accepted.get(i).getPitchDegrees();
                accMiss[i] = accepted.get(i).getMissDistance();
                accTOF[i] = accepted.get(i).getTimeOfFlight();
                accMaxH[i] = accepted.get(i).getMaxHeight();
            }
            Logger.recordOutput("Debug/AcceptedPitches", accPitch);
            Logger.recordOutput("Debug/AcceptedMissDistance", accMiss);
            Logger.recordOutput("Debug/AcceptedTOF", accTOF);
            Logger.recordOutput("Debug/AcceptedMaxHeight", accMaxH);

            List<SolveDebugInfo.CandidateInfo> all = debug.getCandidates();
            double[] allPitch = new double[all.size()];
            double[] allClosest = new double[all.size()];
            String[] allStatus = new String[all.size()];
            for (int i = 0; i < all.size(); i++) {
                allPitch[i] = all.get(i).getPitchDegrees();
                allClosest[i] = all.get(i).getClosestApproach();
                allStatus[i] = all.get(i).getRejection().name();
            }
            Logger.recordOutput("Debug/AllPitches", allPitch);
            Logger.recordOutput("Debug/AllClosest", allClosest);
            Logger.recordOutput("Debug/AllStatus", allStatus);
        } else {
            Logger.recordOutput("Debug/Enabled", false);
        }
    }

    public void periodic() {
        if (trackingEnabled && poseSupplier != null) {
            updateShot();
        }
        // logShotOutput();
        // logTrajectoryDebug();
    }
}
