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
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.FieldLayout;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import edu.wpi.first.wpilibj.Filesystem;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;

public class TrajectoryCalculations {
    private final ShooterSystem hubShooterSystem;
    private final ShooterSystem passLeftShooterSystem;
    private final ShooterSystem passRightShooterSystem;
    private ShooterSystem activeShooterSystem;
    private final TrajectorySolver trajectorySolver;

    public enum ShotGoal {
        HUB,
        PASS_LEFT,
        PASS_RIGHT
    }
    

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
    private boolean DebugMode = Constants.Shooting.TrajectoryCalc.TRAJECTORY_VERBOSITY == SubsystemVerbosity.HIGH;    


    private TrajectoryResult simTrajectoryFallback = null;

    public TrajectoryCalculations() {
        super();

        SolverConstants.setMinTargetDistanceMeters(0.05);
        SolverConstants.setVelocityBufferMultiplier(1.2);
        SolverConstants.setRimClearanceMeters(0.05);
        SolverConstants.setMinEntryAngleDegrees(10.0); 
        SolverConstants.setDragCompensationMultiplier(1.5);

    // Use a tighter pitch search window and the realtime solver mode to reduce CPU usage and lag
    TrajectorySolver.SolverConfig solverConfig = TrajectorySolver.SolverConfig.defaults()
        .toBuilder()
        .minPitchDegrees(Constants.Shooting.TrajectoryCalc.MIN_PITCH_DEG)
        .maxPitchDegrees(Constants.Shooting.TrajectoryCalc.MAX_PITCH_DEG)
        .build();

        GamePiece gamePiece = GamePieces.REBUILT_2026_BALL;

        ShotLookupTable hubTable = loadLookupTable("shot-table-hub.json");
        ShotLookupTable passLeftTable = loadLookupTable("shot-table-pass-left.json");
        ShotLookupTable passRightTable = loadLookupTable("shot-table-pass-right.json");
        System.out.println("Loaded All Lookup Tables");
        
        ShooterConfig shooterConfig = ShooterConfig.builder()
                .pitchLimits(Constants.Shooting.TrajectoryCalc.MIN_PITCH_DEG,
                        Constants.Shooting.TrajectoryCalc.MAX_PITCH_DEG) 
                .rpmLimits(Constants.Shooting.TrajectoryCalc.MIN_RPM,
                        Constants.Shooting.TrajectoryCalc.MAX_RPM) 
                .rpmToVelocityFactor(Constants.Shooting.TrajectoryCalc.RPM_TO_VELOCITY_FACTOR)
                .distanceLimits(Constants.Shooting.TrajectoryCalc.MIN_DISTANCE_M,
                        Constants.Shooting.TrajectoryCalc.MAX_DISTANCE_M)
                .rpmFeedbackThreshold(Constants.Shooting.TrajectoryCalc.RPM_FEEDBACK_THRESHOLD)
                .rpmAbortThreshold(99999.0) 
                .pitchCorrectionPerRpmDeficit(Constants.Shooting.TrajectoryCalc.PITCH_CORRECTION_PER_RPM_DEFICIT)
                .movingCompensationGain(Constants.Shooting.TrajectoryCalc.MOVING_COMPENSATION_GAIN)
                .movingIterations(Constants.Shooting.TrajectoryCalc.MOVING_ITERATIONS)
                .safetyMaxExitVelocity(999.0) // Lowkey just bypass this for now it false flags some times
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
    trajectorySolver.setDebugEnabled(Constants.Shooting.TrajectoryCalc.TRAJECTORY_VERBOSITY == SubsystemVerbosity.HIGH);
    trajectorySolver.setSolveMode(Constants.Shooting.TrajectoryCalc.REALTIME_SOLVER_MODE);

        hubShooterSystem = new ShooterSystem(shooterConfig, hubTable, trajectorySolver);
        hubShooterSystem.setMode(ShotMode.LOOKUP_ONLY);

        passLeftShooterSystem = new ShooterSystem(shooterConfig, passLeftTable, trajectorySolver);
        passLeftShooterSystem.setMode(ShotMode.LOOKUP_ONLY);

        passRightShooterSystem = new ShooterSystem(shooterConfig, passRightTable, trajectorySolver);
        passRightShooterSystem.setMode(ShotMode.LOOKUP_ONLY);

        activeShooterSystem = hubShooterSystem;
    }

    private ShotLookupTable loadLookupTable(String filename) {
        ShotLookupTable table = new ShotLookupTable();
        try {
            File deployDir = Filesystem.getDeployDirectory();
            File file = new File(deployDir, "shooter/" + filename);
            if (file.exists()) {
                ObjectMapper mapper = new ObjectMapper();
                JsonNode root = mapper.readTree(new FileReader(file));
                
                if (root.has("entries") && root.get("entries").isArray()) {
                    ArrayNode array = (ArrayNode) root.get("entries");
                    for (JsonNode entry : array) {
                        double dist = entry.get("distanceMeters").asDouble();
                        double pitch = entry.get("pitchDegrees").asDouble();
                        double rpm = entry.get("rpm").asDouble();
                        if (entry.has("timeOfFlightSeconds")) {
                            double tof = entry.get("timeOfFlightSeconds").asDouble();
                            table.addEntry(dist, pitch, rpm, tof);
                        } else {
                            table.addEntry(dist, pitch, rpm);
                        }
                    }
                    System.out.println("Loaded " + array.size() + " entries from " + filename);
                    Logger.recordOutput("TrajectoryCalculations/Lookup/" + filename + "/Entries", array.size());
                    Logger.recordOutput("TrajectoryCalculations/Lookup/" + filename + "/MinDistance", table.getMinDistance());
                    Logger.recordOutput("TrajectoryCalculations/Lookup/" + filename + "/MaxDistance", table.getMaxDistance());
                } else if (root.isArray()) {
                    ArrayNode array = (ArrayNode) root;
                    for (JsonNode entry : array) {
                        double dist = entry.has("distanceMeters") ? entry.get("distanceMeters").asDouble() : entry.get("distance").asDouble();
                        double pitch = entry.has("pitchDegrees") ? entry.get("pitchDegrees").asDouble() : entry.get("pitch").asDouble();
                        double rpm = entry.get("rpm").asDouble();
                        if (entry.has("timeOfFlightSeconds")) {
                            double tof = entry.get("timeOfFlightSeconds").asDouble();
                            table.addEntry(dist, pitch, rpm, tof);
                        } else if (entry.has("tof")) {
                            double tof = entry.get("tof").asDouble();
                            table.addEntry(dist, pitch, rpm, tof);
                        } else {
                            table.addEntry(dist, pitch, rpm);
                        }
                    }
                    System.out.println("Loaded " + array.size() + " entries from " + filename);
                    Logger.recordOutput("TrajectoryCalculations/Lookup/" + filename + "/Entries", array.size());
                    Logger.recordOutput("TrajectoryCalculations/Lookup/" + filename + "/MinDistance", table.getMinDistance());
                    Logger.recordOutput("TrajectoryCalculations/Lookup/" + filename + "/MaxDistance", table.getMaxDistance());
                }
            } else {
                System.out.println("File not found: " + file.getAbsolutePath());
                Logger.recordOutput("TrajectoryCalculations/Lookup/" + filename + "/Missing", true);
            }
        } catch (IOException e) {
            System.err.println("Failed to load map " + filename + ": " + e.getMessage());
            Logger.recordOutput("TrajectoryCalculations/Lookup/" + filename + "/Error", e.getMessage());
        }
        return table;
    }

    public void setGoal(ShotGoal goal) {
        switch (goal) {
            case HUB:
                activeShooterSystem = hubShooterSystem;
                this.targetSupplier = FieldLayout.ShooterTargets::getAllianceHub;
                break;
            case PASS_LEFT:
                activeShooterSystem = passLeftShooterSystem;
                this.targetSupplier = FieldLayout.ShooterTargets::getAlliancePassLeft;
                break;
            case PASS_RIGHT:
                activeShooterSystem = passRightShooterSystem;
                this.targetSupplier = FieldLayout.ShooterTargets::getAlliancePassRight; 
                break;
        }
    }

    public void setTargetSupplier(Supplier<Translation3d> supplier) {
        this.targetSupplier = supplier;
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
        activeShooterSystem.setMode(mode);
    }

    public ShotMode getMode() {
        return activeShooterSystem.getMode();
    }

    public void setManualOverride(double pitchDegrees, double rpm) {
        activeShooterSystem.setManualOverride(pitchDegrees, rpm);
    }

    public double getNeededYaw() {
        return currentShot.valid ? Math.toDegrees(currentShot.yawAdjustmentRadians) : 0.0;
    }

    public double getNeededPitch() {
        return currentShot.valid ? currentShot.pitchDegrees : 0.0;
    }

    public double getNeededRPM() {
        return currentShot.valid ? currentShot.rpm : 0.0;
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
        return activeShooterSystem;
    }

    public double getLastComputationTimeMs() {
        return lastComputationTimeMs;
    }

    public boolean isReadyToFire() {
        double rpm = currentRPMsupply != null ? currentRPMsupply.get() : 0;
        return activeShooterSystem.isReadyToFire(rpm);
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


    double distToBlueHub = targetPosition.getDistance(FieldLayout.ShooterTargets.kBLUE_HUB_POSE);
    double distToRedHub = targetPosition.getDistance(FieldLayout.ShooterTargets.kRED_HUB_POSE);
        if (Math.min(distToBlueHub, distToRedHub) < 0.5) {
            Pose2d pose = poseSupplier.get();
            if (pose.getX() > FieldLayout.kCenterLineX) {
                targetPosition = FieldLayout.ShooterTargets.kRED_HUB_POSE;
            } else {
                targetPosition = FieldLayout.ShooterTargets.kBLUE_HUB_POSE;
            }
        }
        Pose2d pose = poseSupplier.get();
        Rotation2d rot = pose.getRotation();
        double worldOffsetX = shooterOffset.getX() * rot.getCos() - shooterOffset.getY() * rot.getSin();
        double worldOffsetY = shooterOffset.getX() * rot.getSin() + shooterOffset.getY() * rot.getCos();
        double shooterX = pose.getX() + worldOffsetX;
        double shooterY = pose.getY() + worldOffsetY;
        if (targetPosition.equals(FieldLayout.ShooterTargets.kBLUE_HUB_POSE)
                || targetPosition.equals(FieldLayout.ShooterTargets.kRED_HUB_POSE)
                || targetPosition.equals(FieldLayout.ShooterTargets.kHUB_POSE)) {
            if (pose.getX() > FieldLayout.kCenterLineX) {
                targetPosition = FieldLayout.ShooterTargets.kRED_HUB_POSE;
            } else {
                targetPosition = FieldLayout.ShooterTargets.kBLUE_HUB_POSE;
            }
        }

    double dx = targetPosition.getX() - shooterX;
    double dy = targetPosition.getY() - shooterY;
    double yawRad = Math.atan2(dx, dy);
    lastDistanceMeters = Math.hypot(dx, dy);
    targetYawDegrees = Math.toDegrees(yawRad);

    Logger.recordOutput("Debug/ShooterX", shooterX);
    Logger.recordOutput("Debug/ShooterY", shooterY);
    Logger.recordOutput("Debug/TargetX", targetPosition.getX());
    Logger.recordOutput("Debug/TargetY", targetPosition.getY());
    Logger.recordOutput("Debug/YawRad", yawRad);
    Logger.recordOutput("Debug/TargetYawDegrees", targetYawDegrees);

        double nowMs = Timer.getFPGATimestamp() * 1000.0;
        double distanceChange = Math.abs(lastDistanceMeters - lastSolvedDistance);
        double yawChange = Math.abs(targetYawDegrees - lastSolvedYawDeg);
        boolean enoughTimePassed = (nowMs
                - lastSolveTimestamp) >= Constants.Shooting.TrajectoryCalc.MIN_SOLVE_INTERVAL_MS;

        double vx = 0, vy = 0;
        if (chassisSupplier != null) {
            ChassisSpeeds speeds = chassisSupplier.get();
            vx = speeds.vxMetersPerSecond;
            vy = speeds.vyMetersPerSecond;
        }

        double velMag = Math.hypot(vx, vy);
        boolean inputsChanged = distanceChange > Constants.Shooting.TrajectoryCalc.DISTANCE_CHANGE_THRESHOLD_M
                || yawChange > Constants.Shooting.TrajectoryCalc.YAW_CHANGE_THRESHOLD_DEG
                || velMag > 0.05;

        if (!enoughTimePassed && !inputsChanged && currentShot.valid) {
            Logger.recordOutput("TrajectoryCalc/Skipped", true);
            return;
        }

        double measuredRPM = currentRPMsupply != null ? currentRPMsupply.get() : 0;


        double clampedDistance = Math.max(Constants.Shooting.TrajectoryCalc.MIN_DISTANCE_M, 
                            Math.min(lastDistanceMeters, Constants.Shooting.TrajectoryCalc.MAX_DISTANCE_M));

        ShotInput input = ShotInput.builder()
                        .shooterPositionMeters(shooterX, shooterY, shooterHeightMeters)
                        .shooterYawRadians(yawRad)
                        .targetPositionMeters(targetPosition.getX(), targetPosition.getY(), targetPosition.getZ())
                        .targetRadiusMeters(Constants.Shooting.TrajectoryCalc.TARGET_RADIUS_M)
                        .includeAirResistance(true)
                        .robotVelocity(vx, vy) 
                        .build();

        activeShooterSystem.setSolverInput(input);

    Logger.recordOutput("TrajectoryCalc/TargetDistance", lastDistanceMeters);

        long startTime = System.nanoTime();
        
        currentShot = activeShooterSystem.calculate(clampedDistance, measuredRPM, vx, vy, yawRad);

    if (RobotBase.isSimulation() && currentShot.valid && activeShooterSystem.getMode() == ShotMode.LOOKUP_ONLY
        && Constants.Shooting.TrajectoryCalc.TRAJECTORY_VERBOSITY == SubsystemVerbosity.HIGH) {
            try {
                simTrajectoryFallback = trajectorySolver.solve(input);
            } catch (Exception e) {
                Logger.recordOutput("TrajectoryCalc/SolverError", e.toString());
                simTrajectoryFallback = null;
            }
        } else {
            simTrajectoryFallback = null;
        }

        long endTime = System.nanoTime();
        lastComputationTimeMs = (endTime - startTime) / 1_000_000.0;

        lastSolveTimestamp = nowMs;
        lastSolvedDistance = lastDistanceMeters;
        lastSolvedYawDeg = targetYawDegrees;

        if (loggingEnabled) {
            Logger.recordOutput("TrajectoryCalculations/RPM Required", currentShot.rpm);
            Logger.recordOutput("TrajectoryCalculations/Pitch Required", currentShot.pitchDegrees);
            Logger.recordOutput("TrajectoryCalculations/Yaw Required", targetYawDegrees);
            Logger.recordOutput("TrajectoryCalculations/ComputationTimeMs", lastComputationTimeMs);

        }
    }

    public void calculateShot(double shooterX, double shooterY, double shooterZ,
            double targetX, double targetY, double targetZ) {
        double dx = targetX - shooterX;
        double dy = targetY - shooterY;
        lastDistanceMeters = Math.hypot(dx, dy);
        targetYawDegrees = Math.toDegrees(Math.atan2(dy, dx));
        currentShot = activeShooterSystem.calculate(lastDistanceMeters);
    }

    public void cycleMode() {
        ShotMode[] modes = ShotMode.values();
        int next = (activeShooterSystem.getMode().ordinal() + 1) % modes.length;
        activeShooterSystem.setMode(modes[next]);
        System.out.println("Shot mode: " + modes[next]);
    }

    public void shootBallSim() {
        if (poseSupplier == null) {
            return;
        }

        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds speeds = chassisSupplier != null ? chassisSupplier.get() : new ChassisSpeeds();
        if (!currentShot.valid) {
            System.out.println("Cannot shoot: no valid trajectory or shot parameter");
            return;
        }

        double launchSpeed = currentShot.exitVelocityMps;
        double pitchRad = Math.toRadians(currentShot.pitchDegrees);
        double yawRad = Math.toRadians(targetYawDegrees) + currentShot.yawAdjustmentRadians;

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

    // Do not log this every cycle - only when a new shot is calculated or when this is alot of data so comment out what you dont need 
    // significant changes occur
    // Please dont change this network tables the HTML dashboard relies on these exact keys to display the data correctly, cuz im to lazy to map them to new keys in the dashboard code
    private void logShotOutput() {
        if (!loggingEnabled) {
            return;
        }

        Logger.recordOutput("TrajectoryCalculations/ValidShot", currentShot.valid);
        Logger.recordOutput("TrajectoryCalculations/TargetRPM", currentShot.rpm);
        Logger.recordOutput("TrajectoryCalculations/TargetPitchDeg", currentShot.pitchDegrees);
        Logger.recordOutput("TrajectoryCalculations/TargetYawDeg", targetYawDegrees);
        Logger.recordOutput("TrajectoryCalculations/Distance", lastDistanceMeters);
        Logger.recordOutput("TrajectoryCalculations/ShotSource", currentShot.source.name());
        Logger.recordOutput("TrajectoryCalculations/Mode", activeShooterSystem.getMode().name());
        Logger.recordOutput("TrajectoryCalculations/SourceDetail", activeShooterSystem.getLastSourceDescription());
        Logger.recordOutput("TrajectoryCalculations/InvalidReason", currentShot.invalidReason != null ? currentShot.invalidReason : "None");
        Logger.recordOutput("TrajectoryCalculations/TrackingEnabled", trackingEnabled);
        Logger.recordOutput("TrajectoryCalculations/ExitVelocity", currentShot.exitVelocityMps);
        Logger.recordOutput("TrajectoryCalculations/ComputationTimeMs", lastComputationTimeMs);

        if (currentRPMsupply != null) {
            double measured = currentRPMsupply.get();
            Logger.recordOutput("TrajectoryCalculations/MeasuredRPM", measured);
            Logger.recordOutput("TrajectoryCalculations/RpmDeficit", currentShot.rpm - measured);
            Logger.recordOutput("TrajectoryCalculations/ReadyToFire", activeShooterSystem.isReadyToFire(measured));
        }

        Pose3d goalPose = new Pose3d(targetPosition, new Rotation3d());
        Logger.recordOutput("TrajectoryCalculations/GoalPose3dArray", new Pose3d[] { goalPose });
        Logger.recordOutput("TrajectoryCalculations/TargetX", targetPosition.getX());
        Logger.recordOutput("TrajectoryCalculations/TargetY", targetPosition.getY());
        Logger.recordOutput("TrajectoryCalculations/TargetZ", targetPosition.getZ());
    }

    private void logTrajectoryDebug() {
        if (!loggingEnabled) {
            return;
        }

        TrajectoryResult trajResult = activeShooterSystem.getLastTrajectoryResult();
        if (trajResult == null && simTrajectoryFallback != null) {
            trajResult = simTrajectoryFallback;
        }
        
        if (trajResult == null) {
            Logger.recordOutput("Trajectory/Status", "NONE");
            return;
        }

        Logger.recordOutput("TrajectoryCalculations/Shot/Status", trajResult.getStatus().name());
        Logger.recordOutput("TrajectoryCalculations/Shot/StatusMessage", trajResult.getStatusMessage());
        Logger.recordOutput("TrajectoryCalculations/Shot/Confidence", trajResult.getConfidenceScore());
        Logger.recordOutput("TrajectoryCalculations/Shot/ComputationTimeMs", lastComputationTimeMs);

        if (trajResult.isSuccess()) {
            Logger.recordOutput("TrajectoryCalculations/Shot/PitchDeg", trajResult.getPitchAngleDegrees());
            Logger.recordOutput("TrajectoryCalculations/Shot/YawAdjustDeg", trajResult.getYawAdjustmentDegrees());
            Logger.recordOutput("TrajectoryCalculations/Shot/Velocity", trajResult.getRequiredVelocityMps());
            Logger.recordOutput("TrajectoryCalculations/Shot/TimeOfFlight", trajResult.getTimeOfFlightSeconds());
            Logger.recordOutput("TrajectoryCalculations/Shot/MaxHeight", trajResult.getMaxHeightMeters());
            Logger.recordOutput("TrajectoryCalculations/Shot/Margin", trajResult.getMarginOfErrorMeters());
            Logger.recordOutput("TrajectoryCalculations/Shot/RPM", trajResult.getRecommendedRpm());
            FlywheelSimulator.SimulationResult flywheelSim = trajResult.getFlywheelSimulation();
            if (flywheelSim != null) {
                Logger.recordOutput("TrajectoryCalculations/Flywheel/ExitVelocityMps", flywheelSim.exitVelocityMps);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/MotorPowerPercent", flywheelSim.motorPowerPercent);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/RequiredMotorRpm", flywheelSim.requiredMotorRpm);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/RequiredWheelRpm", flywheelSim.requiredWheelRpm);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/SpinUpTimeSeconds", flywheelSim.spinUpTimeSeconds);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/CurrentDrawAmps", flywheelSim.currentDrawAmps);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/StoredEnergyJoules", flywheelSim.storedEnergyJoules);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/BallSpinRpm", flywheelSim.ballSpinRpm);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/ContactTimeMs", flywheelSim.contactTimeMs);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/EnergyEfficiency", flywheelSim.energyTransferEfficiency);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/SlipRatio", flywheelSim.slipRatio);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/IsAchievable", flywheelSim.isAchievable);
                Logger.recordOutput("TrajectoryCalculations/Flywheel/LimitingFactor", flywheelSim.limitingFactor);
            }

            // Reduced Flight path to fix frame rate issues in advantage scope
            List<Pose3d> flightPath = trajResult.getFlightPath();
            if (!flightPath.isEmpty()) {
                java.util.ArrayList<Pose3d> fullPath = new java.util.ArrayList<>();
                int skipRate = 5; 
                for (int i = 0; i < flightPath.size(); i += skipRate) {
                    fullPath.add(flightPath.get(i));
                }
                Pose3d lastPose = flightPath.get(flightPath.size() - 1);
                if (fullPath.isEmpty() || !fullPath.get(fullPath.size() - 1).equals(lastPose)) {
                    fullPath.add(lastPose);
                }
                if (flightPath.size() > 1) {
                    Pose3d prevPose = flightPath.get(flightPath.size() - 2);
                    double dt = 0.02;
                    double v_x = (lastPose.getX() - prevPose.getX()) / dt;
                    double v_y = (lastPose.getY() - prevPose.getY()) / dt;
                    double v_z = (lastPose.getZ() - prevPose.getZ()) / dt;
                    double curX = lastPose.getX();
                    double curY = lastPose.getY();
                    double curZ = lastPose.getZ();
                    double dtLarge = 0.12; 
                    while (curZ > 0.0 && fullPath.size() < 50) {
                        curX += v_x * dtLarge;
                        curY += v_y * dtLarge;
                        curZ += v_z * dtLarge;
                        v_z -= 9.81 * dtLarge;
                        fullPath.add(new Pose3d(curX, curY, curZ, new Rotation3d(0, Math.atan2(v_z, Math.hypot(v_x, v_y)), Math.atan2(v_y, v_x))));
                    }
                }
                Logger.recordOutput("TrajectoryCalculations/Trajectory/FlightPath",
                        fullPath.toArray(new Pose3d[0]));
                Logger.recordOutput("TrajectoryCalculations/FlightPath", fullPath.toArray(new Pose3d[0]));
            }
        }

        SolveDebugInfo debug = trajResult.getDebugInfo();
        if (debug != null) {
            Logger.recordOutput("TrajectoryCalculations/Debug/TotalTested", debug.getTotalTested());
            Logger.recordOutput("TrajectoryCalculations/Debug/Accepted", debug.getAcceptedCount());
            Logger.recordOutput("TrajectoryCalculations/Debug/TotalRejected", debug.getTotalRejected());
            Logger.recordOutput("TrajectoryCalculations/Debug/RejectedCollision", debug.getRejectedCollisionCount());
            Logger.recordOutput("TrajectoryCalculations/Debug/RejectedArcTooLow", debug.getRejectedArcTooLowCount());
            Logger.recordOutput("TrajectoryCalculations/Debug/RejectedClearance", debug.getRejectedClearanceCount());
            Logger.recordOutput("TrajectoryCalculations/Debug/RejectedMiss", debug.getRejectedMissCount());
            Logger.recordOutput("TrajectoryCalculations/Debug/RejectedFlyover", debug.getRejectedFlyoverCount());
            Logger.recordOutput("TrajectoryCalculations/Debug/BestMissDistance", debug.getBestMissDistance());
            Logger.recordOutput("TrajectoryCalculations/Debug/BestPitchDeg", debug.getBestPitchDegrees());
            Logger.recordOutput("TrajectoryCalculations/Debug/Summary", debug.getSummary());
            Logger.recordOutput("TrajectoryCalculations/Debug/DetailedTable", debug.getDetailedTable());

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
            Logger.recordOutput("TrajectoryCalculations/Debug/AcceptedPitches", accPitch);
            Logger.recordOutput("TrajectoryCalculations/Debug/AcceptedMissDistance", accMiss);
            Logger.recordOutput("TrajectoryCalculations/Debug/AcceptedTOF", accTOF);
            Logger.recordOutput("TrajectoryCalculations/Debug/AcceptedMaxHeight", accMaxH);

            List<SolveDebugInfo.CandidateInfo> all = debug.getCandidates();
            double[] allPitch = new double[all.size()];
            double[] allClosest = new double[all.size()];
            String[] allStatus = new String[all.size()];
            for (int i = 0; i < all.size(); i++) {
                allPitch[i] = all.get(i).getPitchDegrees();
                allClosest[i] = all.get(i).getClosestApproach();
                allStatus[i] = all.get(i).getRejection().name();
            }
            Logger.recordOutput("TrajectoryCalculations/Debug/AllPitches", allPitch);
            Logger.recordOutput("TrajectoryCalculations/Debug/AllClosest", allClosest);
            Logger.recordOutput("TrajectoryCalculations/Debug/AllStatus", allStatus);
        } else {
            Logger.recordOutput("TrajectoryCalculations/Debug/Enabled", false);
        }
    }

    public void periodic() {
        if (trackingEnabled && poseSupplier != null) {
            updateShot();
        }


        if (DebugMode) {
            logShotOutput();
            logTrajectoryDebug();
        }
    }
}
