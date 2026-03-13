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

    private final TalonFX m_driveMotor;
    private final CANcoder m_canCoder1;
    private final CANcoder m_canCoder2;

    private double m_targetDegWrapped = 0.0;
    private double m_targetDegUnWrapped = Constants.Shooting.Turret.TURRET_START_ANGLE;
    private double m_currentDegWrapped = 0.0;
    private double m_currentDegUnWrapped = Constants.Shooting.Turret.TURRET_START_ANGLE;

    private double m_encoderOffset = 0;

    private final static boolean CANCODER_TEST = false;

    private final SubsystemVerbosity verbosity;

    public final static ArmFeedforward feedforward = Constants.Shooting.Turret.feedforward;

    public final static ProfiledPIDController pidController = Constants.Shooting.Turret.pidController;

    private Supplier<Double> simSupplier;
    private double voltage;

    public TurretSubsystem() {
        m_driveMotor = new TalonFX(Ports.Shooting.Turret.kTurretMotorId);
        m_canCoder1 = new CANcoder(Ports.Shooting.Turret.kCanCoder1Id);
        m_canCoder2 = new CANcoder(Ports.Shooting.Turret.kCanCoder2Id);

        updateAngle();
        m_encoderOffset = getAngleUnWrapped() - Constants.Shooting.Turret.TURRET_START_ANGLE; // Starts at 360 deg
        if (CANCODER_TEST) {
            m_encoderOffset = calculateEncoderAngle();
        } else {
            calculateEncoderAngle();
        }

        if (Robot.isSimulation()) { // Brute force Sim
            pidController.setP(0.3);
        }

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_driveMotor.getConfigurator().apply(driveConfig);

        pidController.reset(m_currentDegUnWrapped);

        verbosity = SubsystemVerbosity.HIGH;
    }

    public double getAngleWrapped() {
        return m_currentDegWrapped;
    }

    public double getAngleUnWrapped() {
        return m_currentDegUnWrapped;
    }

    public double calculateEncoderAngle() {
        double enc1 = m_canCoder1.getAbsolutePosition().getValueAsDouble();
        double enc2 = m_canCoder2.getAbsolutePosition().getValueAsDouble();

        Logger.recordOutput("Subsystems/Turret/ENCODER 1", enc1);
        Logger.recordOutput("Subsystems/Turret/ENCODER 2", enc2);

        double diff = (enc1 - enc2) % 1.0;
        double coarseRotations = diff * Constants.Shooting.Turret.PERIOD;

        double n1 = Math.round((coarseRotations * Constants.Shooting.Turret.GEAR_RATIO_1) - enc1);

        double preciseRotations = (n1 + enc1) / Constants.Shooting.Turret.GEAR_RATIO_1;

        double posDegrees = coarseRotations * 360;
        Logger.recordOutput("Subsystems/Turret/Encoder Calculated Angle", posDegrees);
        return posDegrees;
    }

    public void setSimSupplier(Supplier<Double> supplier) {
        simSupplier = supplier;
    }

    public double getVoltage() {
        return voltage;
    }

    public void updateAngle() {
        if (Robot.isSimulation()) {
            Logger.recordOutput("Subsystems/Turret/SimMode", 1);
            Logger.recordOutput("Subsystems/Turret/SimSupplierNull", simSupplier == null ? 1 : 0);
            if (simSupplier == null) {
                m_currentDegUnWrapped = 360;
            } else {
                double simDeg = simSupplier.get();
                m_currentDegUnWrapped = simDeg;
                Logger.recordOutput("Subsystems/Turret/SimSupplierDegrees", simDeg);
            }
            m_currentDegWrapped = inputModulus(m_currentDegUnWrapped, 0.0,
                    Constants.Shooting.Turret.FULL_REVOLUTION_DEG, Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
        } else {
            double rawAngle = m_driveMotor.getPosition().getValueAsDouble() * Constants.Shooting.Turret.GEAR_RATIO_MOTOR
                    * 360;
            m_currentDegUnWrapped = rawAngle - m_encoderOffset;
            m_currentDegWrapped = inputModulus(m_currentDegUnWrapped, 0.0,
                    Constants.Shooting.Turret.FULL_REVOLUTION_DEG, Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
        }
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
        if (diff > modulus / 2.0) {
            diff -= modulus;
        } else if (diff < -modulus / 2.0) {
            diff += modulus;
        }
        return diff;
    }

    public void setTarget(double degrees) {
        double wrappedTarget = inputModulus(degrees, 0.0,
                Constants.Shooting.Turret.FULL_REVOLUTION_DEG, Constants.Shooting.Turret.FULL_REVOLUTION_DEG);

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
                Constants.Shooting.Turret.FULL_REVOLUTION_DEG, Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
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

    public void aimAtPoint(Translation3d fieldTarget) {
        Pose2d robotPose = robotPoseSupplier.get();
        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        double turretAngleDeg = fieldAngleDeg - robotHeadingDeg;
        setTarget(turretAngleDeg);
    }

    public double getHubAngle(Translation3d fieldTarget) {
        Pose2d robotPose = robotPoseSupplier.get();
        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        double turretAngleDeg = fieldAngleDeg - robotHeadingDeg;
        return turretAngleDeg;
    }

    public Command aimAtPointCommand(Translation3d fieldTarget) {
        return run(() -> aimAtPoint(fieldTarget));
    }

    private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

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

    private Pose3d getTurretPose() {
        return new Pose3d(0.1362075, 0, 0.3370134992, new Rotation3d(0, 0, Math.toRadians(180 + getAngleWrapped())));
    }

    @Override
    public void periodic() {
        updateAngle();

        double pidOutput = pidController.calculate(m_currentDegUnWrapped, m_targetDegUnWrapped);

        double ffOutput = feedforward.calculate(pidController.getSetpoint().position,
                pidController.getSetpoint().velocity);

        voltage = pidOutput + ffOutput;

        m_driveMotor.setVoltage(voltage);

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
            Logger.recordOutput("Subsystems/Turret/Motor Voltage", m_driveMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Turret/Motor Temperature", m_driveMotor.getDeviceTemp().getValueAsDouble());
            Logger.recordOutput("Subsystems/Turret/Current", m_driveMotor.getStatorCurrent().getValueAsDouble());
            Logger.recordOutput("Subsystems/Turret/Velocity", m_driveMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("Subsystems/Turret/Setpoint Angle", pidController.getSetpoint().position);
            Logger.recordOutput("Subsystems/Turret/Setpoint Velocity", pidController.getSetpoint().velocity);
        }
    }
}
