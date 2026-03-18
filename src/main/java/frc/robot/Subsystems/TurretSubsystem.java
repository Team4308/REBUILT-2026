package frc.robot.Subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.Util.SubsystemVerbosity;

public class TurretSubsystem extends SubsystemBase {

    // ── Calibration state ────────────────────────────────────────────────────────
    private enum CalibrationState {
        /** Still running CANcoder math to find the true angle. */
        CALIBRATING,
        /** CANcoder angle confirmed; motor encoder is now the sole source. */
        CALIBRATED
    }

    private CalibrationState m_calibrationState = CalibrationState.CALIBRATING;

    /**
     * How many consecutive periodic cycles the coarse + n1 solution must be
     * stable before we commit to the motor encoder.
     */
    private static final int STABLE_CYCLES_REQUIRED = 20;
    private int m_stableCycles = 0;

    // ── Hardware ─────────────────────────────────────────────────────────────────
    private final TalonFX m_driveMotor;
    private final CANcoder m_canCoder1;
    private final CANcoder m_canCoder2;

    // ── Angle tracking ───────────────────────────────────────────────────────────
    private double m_targetDegWrapped = 0.0;
    private double m_targetDegUnWrapped = Constants.Shooting.Turret.TURRET_START_ANGLE;
    private double m_currentDegWrapped = 0.0;
    private double m_currentDegUnWrapped = Constants.Shooting.Turret.TURRET_START_ANGLE;

    /**
     * Offset applied to the motor encoder reading (set once calibration completes).
     */
    private double m_motorEncoderOffset = 0.0;

    // ── CANcoder algorithm state (used only during CALIBRATING) ──────────────────
    private double m_lastDiff = Double.NaN;
    private double m_lastCoarseRotations = Double.NaN;
    private double m_lastN1 = Double.NaN;
    private int m_n1Candidate = 0;
    private int m_n1CandidateCycles = 0;
    private int m_warmupCycles = 0;

    private static final int WARMUP_CYCLES = 10;
    private static final double MAX_DELTA_ROTATIONS = 20.0 / 360.0;
    private static final int N1_CONFIRM_CYCLES = 3;

    // ── Controllers ──────────────────────────────────────────────────────────────
    public final static ArmFeedforward feedforward = Constants.Shooting.Turret.feedforward;
    public final static ProfiledPIDController pidController = Constants.Shooting.Turret.pidController;

    // ── Sim suppliers
    // ─────────────────────────────────────────────────────────────
    private Supplier<Double> simSupplier;
    private Supplier<Double> enc1SimSupplier;
    private Supplier<Double> enc2SimSupplier;

    // ── Misc ─────────────────────────────────────────────────────────────────────
    private double voltage;
    private boolean enabled;
    private final SubsystemVerbosity verbosity;
    private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

    // ─────────────────────────────────────────────────────────────────────────────

    public TurretSubsystem(boolean enabled) {
        m_driveMotor = new TalonFX(Ports.Shooting.Turret.kTurretMotorId);
        m_canCoder1 = new CANcoder(Ports.Shooting.Turret.kCanCoder1Id);
        m_canCoder2 = new CANcoder(Ports.Shooting.Turret.kCanCoder2Id);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_driveMotor.getConfigurator().apply(driveConfig);

        if (Robot.isSimulation()) {
            pidController.setP(0.3);
            // Sim never needs CANcoder calibration; go straight to CALIBRATED.
            m_calibrationState = CalibrationState.CALIBRATED;
            m_currentDegUnWrapped = Constants.Shooting.Turret.TURRET_START_ANGLE;
        }

        pidController.reset(m_currentDegUnWrapped);

        verbosity = SubsystemVerbosity.HIGH;
        this.enabled = enabled;
    }

    // ── Public angle getters
    // ──────────────────────────────────────────────────────

    public double getAngleWrapped() {
        return m_currentDegWrapped;
    }

    public double getAngleUnWrapped() {
        return m_currentDegUnWrapped;
    }

    public boolean isCalibrated() {
        return m_calibrationState == CalibrationState.CALIBRATED;
    }

    // ── Sim wiring
    // ────────────────────────────────────────────────────────────────

    public void setSimSupplier(Supplier<Double> angleSupplier,
            Supplier<Double> enc1Supplier,
            Supplier<Double> enc2Supplier) {
        simSupplier = angleSupplier;
        enc1SimSupplier = enc1Supplier;
        enc2SimSupplier = enc2Supplier;
    }

    public double getVoltage() {
        return voltage;
    }

    // ── Angle update ─────────────────────────────────────────────────────────────

    /**
     * Called every periodic cycle.
     * During CALIBRATING: runs the dual-CANcoder algorithm and checks for
     * stability; when stable, latches the motor encoder offset and transitions
     * to CALIBRATED.
     * During CALIBRATED: reads only the motor encoder (cheap, deterministic).
     */
    public void updateAngle() {
        if (Robot.isSimulation()) {
            m_currentDegUnWrapped = (simSupplier == null) ? 360.0 : simSupplier.get();
            if (simSupplier != null)
                Logger.recordOutput("Subsystems/Turret/SimSupplierDegrees", m_currentDegUnWrapped);
            m_currentDegWrapped = inputModulus(m_currentDegUnWrapped, 0.0,
                    Constants.Shooting.Turret.FULL_REVOLUTION_DEG,
                    Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
            return;
        }

        if (m_calibrationState == CalibrationState.CALIBRATING) {
            // ── Run the CANcoder algorithm ──────────────────────────────────────
            double cancoderAngle = calculateEncoderAngle();

            // Check stability: warmup complete + n1 has been rock-steady
            // (m_n1CandidateCycles == 0 means rawN1Int matched m_lastN1 this cycle)
            boolean warmedUp = m_warmupCycles >= WARMUP_CYCLES;
            boolean n1Stable = !Double.isNaN(m_lastN1) && (m_n1CandidateCycles == 0);

            if (warmedUp && n1Stable) {
                m_stableCycles++;
            } else {
                m_stableCycles = 0;
            }

            // Update current angle from CANcoder while still calibrating
            m_currentDegUnWrapped = cancoderAngle - Constants.Shooting.Turret.TURRET_OFFSET_ANGLE;
            m_currentDegWrapped = inputModulus(m_currentDegUnWrapped, 0.0,
                    Constants.Shooting.Turret.FULL_REVOLUTION_DEG,
                    Constants.Shooting.Turret.FULL_REVOLUTION_DEG);

            Logger.recordOutput("Subsystems/Turret/Calibration/StableCycles", m_stableCycles);
            Logger.recordOutput("Subsystems/Turret/Calibration/State", "CALIBRATING");

            if (m_stableCycles >= STABLE_CYCLES_REQUIRED) {
                // ── Commit: compute motor encoder offset and switch modes ───────
                double motorRotations = m_driveMotor.getPosition().getValueAsDouble();
                double motorDeg = motorRotations * Constants.Shooting.Turret.GEAR_RATIO_MOTOR * 360.0;
                m_motorEncoderOffset = motorDeg - m_currentDegUnWrapped;

                m_calibrationState = CalibrationState.CALIBRATED;

                Logger.recordOutput("Subsystems/Turret/Calibration/State", "CALIBRATED");
                Logger.recordOutput("Subsystems/Turret/Calibration/OffsetDeg", m_motorEncoderOffset);
                Logger.recordOutput("Subsystems/Turret/Calibration/LatchedAngle", m_currentDegUnWrapped);
            }

        } else {
            // ── CALIBRATED: pure motor encoder path ────────────────────────────
            double motorRotations = m_driveMotor.getPosition().getValueAsDouble();
            double motorDeg = motorRotations * Constants.Shooting.Turret.GEAR_RATIO_MOTOR * 360.0;

            m_currentDegUnWrapped = motorDeg - m_motorEncoderOffset;
            m_currentDegWrapped = inputModulus(m_currentDegUnWrapped, 0.0,
                    Constants.Shooting.Turret.FULL_REVOLUTION_DEG,
                    Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
        }
    }

    // ── CANcoder dual-encoder algorithm (runs only during CALIBRATING) ───────────

    /**
     * Computes the precise absolute angle (degrees) from the two CANcoders.
     * This is an expensive call; it is intentionally invoked only while
     * {@link CalibrationState#CALIBRATING}.
     */
    private double calculateEncoderAngle() {
        double enc1, enc2;
        if (Robot.isReal()) {
            enc1 = m_canCoder1.getAbsolutePosition().getValueAsDouble();
            enc2 = m_canCoder2.getAbsolutePosition().getValueAsDouble();
        } else {
            enc1 = (enc1SimSupplier != null) ? enc1SimSupplier.get() : 0.0;
            enc2 = (enc2SimSupplier != null) ? enc2SimSupplier.get() : 0.0;
        }

        Logger.recordOutput("Subsystems/Turret/ENCODER 1", enc1);
        Logger.recordOutput("Subsystems/Turret/ENCODER 2", enc2);

        double rawDiff = enc1 - enc2;
        double diff = rawDiff - Math.floor(rawDiff + 0.5);

        if (!Double.isNaN(m_lastDiff)) {
            double delta = diff - m_lastDiff;
            delta = delta - Math.floor(delta + 0.5);
            diff = m_lastDiff + delta;
        }

        double coarseRotations = diff * Constants.Shooting.Turret.PERIOD;

        boolean warmingUp = m_warmupCycles < WARMUP_CYCLES;
        if (!warmingUp && !Double.isNaN(m_lastCoarseRotations) &&
                Math.abs(coarseRotations - m_lastCoarseRotations) > MAX_DELTA_ROTATIONS) {
            coarseRotations = m_lastCoarseRotations;
            m_lastDiff = m_lastCoarseRotations / Constants.Shooting.Turret.PERIOD;
        } else {
            m_lastDiff = diff;
            m_lastCoarseRotations = coarseRotations;
            m_warmupCycles++;
        }

        double rawN1 = (coarseRotations * Constants.Shooting.Turret.GEAR_RATIO_1) - enc1;
        int rawN1Int = (int) Math.round(rawN1);
        double n1;

        if (Double.isNaN(m_lastN1)) {
            n1 = rawN1Int;
            m_n1Candidate = rawN1Int;
            m_n1CandidateCycles = 0;
        } else if (rawN1Int == (int) m_lastN1) {
            n1 = (int) m_lastN1;
            m_n1Candidate = rawN1Int;
            m_n1CandidateCycles = 0;
        } else if (rawN1Int == m_n1Candidate) {
            m_n1CandidateCycles++;
            if (m_n1CandidateCycles >= N1_CONFIRM_CYCLES) {
                n1 = rawN1Int;
                m_n1CandidateCycles = 0;
            } else {
                n1 = (int) m_lastN1;
            }
        } else {
            m_n1Candidate = rawN1Int;
            m_n1CandidateCycles = 1;
            n1 = (int) m_lastN1;
        }

        m_lastN1 = n1;

        Logger.recordOutput("Subsystems/Turret/rawN1_frac", rawN1 - Math.floor(rawN1 + 0.5));
        Logger.recordOutput("Subsystems/Turret/n1Candidate", m_n1Candidate);
        Logger.recordOutput("Subsystems/Turret/n1CandidateCycles", m_n1CandidateCycles);

        double preciseRotations = (n1 + enc1) / Constants.Shooting.Turret.GEAR_RATIO_1;
        double coarseDeg = coarseRotations * 360.0;

        Logger.recordOutput("Subsystems/Turret/Encoder Calculated Angle (Coarse)", coarseDeg);
        Logger.recordOutput("Subsystems/Turret/Encoder Calculated Angle (Precise)", preciseRotations * 360.0);

        return preciseRotations * 360.0;
    }

    // ── Targeting
    // ─────────────────────────────────────────────────────────────────

    private double inputModulus(double value, double min, double max, double modulus) {
        double wrappedValue = (value - min) % modulus;
        if (wrappedValue < 0)
            wrappedValue += modulus;
        return wrappedValue + min;
    }

    private double getWrappedError(double currentDeg, double targetDeg) {
        double modulus = Constants.Shooting.Turret.FULL_REVOLUTION_DEG;
        double diff = (targetDeg - currentDeg) % modulus;
        if (diff > modulus / 2.0)
            diff -= modulus;
        else if (diff < -modulus / 2.0)
            diff += modulus;
        return diff;
    }

    public void setTarget(double degrees) {
        double wrappedTarget = inputModulus(degrees, 0.0,
                Constants.Shooting.Turret.FULL_REVOLUTION_DEG,
                Constants.Shooting.Turret.FULL_REVOLUTION_DEG);

        int kMin = (int) Math.ceil((Constants.Shooting.Turret.MIN_DEGREES - wrappedTarget) / 360.0);
        int kMax = (int) Math.floor((Constants.Shooting.Turret.MAX_DEGREES - wrappedTarget) / 360.0);

        double closestTarget = wrappedTarget;
        double minDistance = Double.MAX_VALUE;

        for (int k = kMin; k <= kMax; k++) {
            double candidate = wrappedTarget + k * 360.0;
            double distance = Math.abs(m_currentDegUnWrapped - candidate);
            if (distance < minDistance) {
                minDistance = distance;
                closestTarget = candidate;
            }
        }

        m_targetDegUnWrapped = closestTarget;
        m_targetDegWrapped = inputModulus(m_targetDegUnWrapped, 0.0,
                Constants.Shooting.Turret.FULL_REVOLUTION_DEG,
                Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
    }

    public boolean isAtTarget() {
        double wrappedError = getWrappedError(m_currentDegWrapped, m_targetDegWrapped);
        return Math.abs(wrappedError) <= Constants.Shooting.Turret.TURRET_TOLERANCE_DEGREES
                && m_driveMotor.getVelocity().getValueAsDouble() < Constants.Shooting.Turret.STOPPED_VELOCITY;
    }

    public Command moveToTarget(Supplier<Double> degrees) {
        return run(() -> setTarget(degrees.get())).until(this::isAtTarget);
    }

    public Command moveToTarget(Supplier<Double> degrees, double timeoutMs) {
        return run(() -> setTarget(degrees.get())).until(this::isAtTarget).withTimeout(timeoutMs / 1000.0);
    }

    public void resetTurret() {
        setTarget(0.0);
    }

    public Command resetTurretCommand() {
        return run(this::resetTurret).until(this::isAtTarget);
    }

    // ── Aiming helpers
    // ────────────────────────────────────────────────────────────

    public void aimAtPoint(Translation3d fieldTarget) {
        Pose2d robotPose = robotPoseSupplier.get();
        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();
<<<<<<< HEAD
    // Compute the required turret rotation relative to the robot heading.
    double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
    double robotHeadingDeg = robotPose.getRotation().getDegrees();

    // Convert the field-relative angle to a robot-relative angle.
    // Rotation2d.minus() gives the angle that must be added to the robot heading to point at the field angle.
    double relativeAngleDeg = Rotation2d.fromDegrees(fieldAngleDeg).minus(robotPose.getRotation()).getDegrees();

    // Apply sign/flip if turret encoders rotate in the opposite direction of the field frame.
    double turretAngleDeg = Constants.Shooting.Turret.TURRET_AIM_SIGN * relativeAngleDeg;
    turretAngleDeg += (Constants.Shooting.Turret.TURRET_OFFSET_ANGLE % 360.0);

    // Keep angle within [0, 360) before handing it to setTarget.
    turretAngleDeg = ((turretAngleDeg % 360.0) + 360.0) % 360.0;
        setTarget(turretAngleDeg);

        if (verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Turret/AimAtPoint/TargetTurretDeg", turretAngleDeg);
        }
=======
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        setTarget(fieldAngleDeg - robotHeadingDeg);
>>>>>>> f471488262c98f5373065973b7135d0a7517b531
    }

    public double getHubAngle(Translation3d fieldTarget) {
        Pose2d robotPose = robotPoseSupplier.get();
        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();
<<<<<<< HEAD
    double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
    double robotHeadingDeg = robotPose.getRotation().getDegrees();
    double turretAngleDeg = fieldAngleDeg - Constants.Shooting.Turret.TURRET_AIM_SIGN * robotHeadingDeg;
    turretAngleDeg += (Constants.Shooting.Turret.TURRET_OFFSET_ANGLE % 360.0);

    turretAngleDeg = ((turretAngleDeg % 360.0) + 360.0) % 360.0;
        return turretAngleDeg;
=======
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        return fieldAngleDeg - robotHeadingDeg;
>>>>>>> f471488262c98f5373065973b7135d0a7517b531
    }

    public Command aimAtPointCommand(Translation3d fieldTarget) {
        return run(() -> aimAtPoint(fieldTarget));
    }

    public void setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.robotPoseSupplier = poseSupplier;
    }

    public void aimAtPassingZone(Pose2d target) {
        Pose2d robotPose = robotPoseSupplier.get();
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double globalAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = robotPose.getRotation().getDegrees();
        setTarget(globalAngleToTarget - robotHeading);
    }

    public void aimAtPassingSide() {
        setTarget(100);
    }

    public Command aimAtPassingZoneCommand(Pose2d target) {
        return run(() -> aimAtPassingZone(target));
    }

    public void stopMotors() {
        m_driveMotor.setVoltage(0);
    }

    // ── Pose for AdvantageKit
    // ─────────────────────────────────────────────────────

    private Pose3d getTurretPose() {
        return new Pose3d(-0.1362075, 0, 0.3370134992,
                new Rotation3d(0, 0, Math.toRadians(-getAngleWrapped())));
    }

    // ── Periodic
    // ──────────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        updateAngle(); // CANcoder during CALIBRATING, motor encoder after

        double pidOutput = pidController.calculate(m_currentDegUnWrapped, m_targetDegUnWrapped);
        double ffOutput = feedforward.calculate(
                pidController.getSetpoint().position,
                pidController.getSetpoint().velocity);

        voltage = pidOutput + ffOutput;

        if (enabled)
            m_driveMotor.setVoltage(voltage);

        // ── Logging ──────────────────────────────────────────────────────────────
        if (verbosity == SubsystemVerbosity.LOW || verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Turret/Is At Target?", isAtTarget());
            Logger.recordOutput("Subsystems/Turret/Angle (Wrapped)", m_currentDegWrapped);
            Logger.recordOutput("Subsystems/Turret/Angle (Unwrapped)", m_currentDegUnWrapped);
            Logger.recordOutput("Subsystems/Turret/Target (Wrapped)", m_targetDegWrapped);
            Logger.recordOutput("Subsystems/Turret/Degree Error",
                    Math.abs(getWrappedError(m_currentDegWrapped, m_targetDegWrapped)));
            Logger.recordOutput("Subsystems/Turret/Pose", getTurretPose());
            Logger.recordOutput("Subsystems/Turret/Calibrated", isCalibrated());
        }

        if (verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Turret/Target (Unwrapped)", m_targetDegUnWrapped);
            Logger.recordOutput("Subsystems/Turret/PID Output", pidOutput);
            Logger.recordOutput("Subsystems/Turret/FF Output", ffOutput);
            Logger.recordOutput("Subsystems/Turret/Applied Voltage", voltage);
            Logger.recordOutput("Subsystems/Turret/Motor Voltage",
                    m_driveMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Turret/Motor Temperature",
                    m_driveMotor.getDeviceTemp().getValueAsDouble());
            Logger.recordOutput("Subsystems/Turret/Current",
                    m_driveMotor.getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("Subsystems/Turret/Velocity",
                    m_driveMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("Subsystems/Turret/Setpoint Angle",
                    pidController.getSetpoint().position);
            Logger.recordOutput("Subsystems/Turret/Setpoint Velocity",
                    pidController.getSetpoint().velocity);
        }
    }
}