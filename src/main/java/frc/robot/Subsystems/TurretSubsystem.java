package frc.robot.Subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.Util.SubsystemVerbosity;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX m_driveMotor;

    private double m_targetDegWrapped = 0.0;
    private double m_targetDegUnWrapped = Constants.Shooting.Turret.TURRET_START_ANGLE;
    private double m_currentDegWrapped = 0.0;
    private double m_currentDegUnWrapped = Constants.Shooting.Turret.TURRET_START_ANGLE;

    private double m_motorEncoderOffset = -Constants.Shooting.Turret.TURRET_START_ANGLE;

    public final static ArmFeedforward feedforward = Constants.Shooting.Turret.feedforward;
    public final static ProfiledPIDController pidController = Constants.Shooting.Turret.pidController;

    private Supplier<Double> simSupplier;

    private double voltage;
    private boolean enabled;
    private final SubsystemVerbosity verbosity;

    public TurretSubsystem(boolean enabled) {
        m_driveMotor = new TalonFX(Ports.Shooting.Turret.kTurretMotorId);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_driveMotor.getConfigurator().apply(driveConfig);

        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;
        limitConfigs.SupplyCurrentLimit = 60;
        limitConfigs.SupplyCurrentLimitEnable = true;
        m_driveMotor.getConfigurator().apply(limitConfigs);

        if (Robot.isSimulation()) {
            pidController.setP(0.3);
            m_currentDegUnWrapped = Constants.Shooting.Turret.TURRET_START_ANGLE;
        }
        m_currentDegUnWrapped = Constants.Shooting.Turret.TURRET_START_ANGLE;

        verbosity = SubsystemVerbosity.LOW;
        this.enabled = enabled;
        m_driveMotor.setPosition(0);

    }

    public double getAngleWrapped() {
        return m_currentDegWrapped;
    }

    public double getAngleUnWrapped() {
        return m_currentDegUnWrapped;
    }

    public void setSimSupplier(Supplier<Double> angleSupplier) {
        simSupplier = angleSupplier;
    }

    public double getVoltage() {
        return voltage;
    }

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

        double motorRotations = m_driveMotor.getPosition().getValueAsDouble();
        double motorDeg = motorRotations * Constants.Shooting.Turret.GEAR_RATIO_MOTOR * 360.0;

        m_currentDegUnWrapped = motorDeg - m_motorEncoderOffset;
        m_currentDegWrapped = inputModulus(m_currentDegUnWrapped, 0.0,
                Constants.Shooting.Turret.FULL_REVOLUTION_DEG,
                Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
    }

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

    public boolean isAtTargetAtRest() {
        double wrappedError = getWrappedError(m_currentDegWrapped, m_targetDegWrapped);
        return Math.abs(wrappedError) <= Constants.Shooting.Turret.TURRET_TOLERANCE_DEGREES
                && Math.abs(m_driveMotor.getVelocity().getValueAsDouble()) < Constants.Shooting.Turret.STOPPED_VELOCITY;
    }

    public boolean isAtTarget() {
        double wrappedError = getWrappedError(m_currentDegWrapped, m_targetDegWrapped);
        return Math.abs(wrappedError) <= Constants.Shooting.Turret.TURRET_TOLERANCE_DEGREES;
    }

    public Command moveToTarget(Supplier<Double> degrees) {
        return run(() -> setTarget(degrees.get()));
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

    public void stopMotors() {
        m_driveMotor.setVoltage(0);
    }

    private Pose3d getTurretPose() {
        return new Pose3d(-0.1362075, 0, 0.3370134992,
                new Rotation3d(0, 0, Math.toRadians(getAngleWrapped())));
    }

    @Override
    public void periodic() {
        updateAngle();

        m_targetDegUnWrapped = DoubleUtils.clamp(m_targetDegUnWrapped, Constants.Shooting.Turret.MIN_DEGREES,
                Constants.Shooting.Turret.MAX_DEGREES);

        double pidOutput = pidController.calculate(m_currentDegUnWrapped, m_targetDegUnWrapped);
        double ffOutput = feedforward.calculate(
                pidController.getSetpoint().position,
                pidController.getSetpoint().velocity,
                Units.degreesToRadians(Constants.Shooting.Turret.MAX_ACCELERATION));

        if (ffOutput == 0 && !isAtTarget()) {
            if (pidOutput < 0) {
                pidOutput -= 0.25;
            } else if (pidOutput > 0) {
                pidOutput += 0.25;
            }
        }

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