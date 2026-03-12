package frc.robot.Subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private final TalonFX driveMotor;
    private final CANcoder canCoder1;
    private final CANcoder canCoder2;

    private double targetDegWrapped = 0.0;
    private double targetDegUnWrapped = 0.0;
    private double currentDegWrapped = 0.0;
    private double currentDegUnWrapped = 0.0;

    private double encoderOffset = 0;

    private final static boolean CANCODER_TEST = false;

    public final static ArmFeedforward feedforward = new ArmFeedforward(0.24, 0, 0.0075, 0.01);

    public final static ProfiledPIDController pidController = new ProfiledPIDController(
            0.035, 0.0, 0.0,
            new TrapezoidProfile.Constraints(1500, 2000));

    public TurretSubsystem() {
        driveMotor = new TalonFX(Constants.Shooting.Turret.DRIVE_MOTOR_ID);
        canCoder1 = new CANcoder(Constants.Shooting.Turret.CANCODER1_ID);
        canCoder2 = new CANcoder(Constants.Shooting.Turret.CANCODER2_ID);

        updateAngle();
        encoderOffset = getAngleUnWrapped();
        if (CANCODER_TEST) {
            encoderOffset = calculateEncoderAngle();
        } else {
            calculateEncoderAngle();
        }

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveConfig);
    }

    public double getAngleWrapped() {
        return currentDegWrapped;
    }

    public double getAngleUnWrapped() {
        return currentDegUnWrapped;
    }

    public double calculateEncoderAngle() {
        double enc1 = canCoder1.getAbsolutePosition().getValueAsDouble();
        double enc2 = canCoder2.getAbsolutePosition().getValueAsDouble();

        Logger.recordOutput("Subsystems/Turret/ENCODER 1", enc1);
        Logger.recordOutput("Subsystems/Turret/ENCODER 2", enc2);

        double diff = (enc1 % enc2) % 1.0;
        double coarseRotations = diff * Constants.Shooting.Turret.PERIOD;

        double n1 = Math.round((coarseRotations * Constants.Shooting.Turret.GEAR_RATIO_1) - enc1);

        double preciseRotations = (n1 + enc1) / Constants.Shooting.Turret.GEAR_RATIO_1;

        double posDegrees = coarseRotations * 360;
        Logger.recordOutput("Subsystems/Turret/Encoder Calculated Angle", posDegrees);
        return posDegrees;
    }

    public void updateAngle() {
        double rawAngle = driveMotor.getPosition().getValueAsDouble() * Constants.Shooting.Turret.GEAR_RATIO_MOTOR
                * 360;
        currentDegUnWrapped = rawAngle - encoderOffset;
    currentDegWrapped = inputModulus(currentDegUnWrapped, 0.0,
        Constants.Shooting.Turret.FULL_REVOLUTION_DEG, Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
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
            double distance = Math.abs(currentDegUnWrapped - candidate);
            if (distance < minDistance) {
                minDistance = distance;
                closestTarget = candidate;
            }
        }

        targetDegUnWrapped = closestTarget;
        targetDegWrapped = inputModulus(targetDegUnWrapped, 0.0,
                Constants.Shooting.Turret.FULL_REVOLUTION_DEG, Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
    }

    public boolean isAtTarget() {
    double wrappedError = getWrappedError(currentDegWrapped, targetDegWrapped);
    return Math.abs(wrappedError) <= Constants.Shooting.Turret.TURRET_TOLERANCE_DEGREES
        && driveMotor.getVelocity().getValueAsDouble() < Constants.Shooting.Turret.STOPPED_VELOCITY;

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
        driveMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        updateAngle();

        double pidOutput = pidController.calculate(currentDegUnWrapped, targetDegUnWrapped);

        double ffOutput = feedforward.calculate(pidController.getSetpoint().position,
                pidController.getSetpoint().velocity);

        double voltage = pidOutput + ffOutput;

        driveMotor.setVoltage(voltage);

        Logger.recordOutput("Subsystems/Turret/Angle (wrapped)", currentDegWrapped);
        Logger.recordOutput("Subsystems/Turret/Angle (unwrapped)", currentDegUnWrapped);
        Logger.recordOutput("Subsystems/Turret/Target (unwrapped)", targetDegUnWrapped);
        Logger.recordOutput("Subsystems/Turret/Target (wrapped)", targetDegWrapped);
        Logger.recordOutput("Subsystems/Turret/Profile Setpoint", pidController.getSetpoint().position);
        Logger.recordOutput("Subsystems/Turret/Profile Velocity", pidController.getSetpoint().velocity);
        Logger.recordOutput("Subsystems/Turret/PID Output", pidOutput);
        Logger.recordOutput("Subsystems/Turret/FF Output", ffOutput);
        Logger.recordOutput("Subsystems/Turret/Voltage", voltage);
        Logger.recordOutput("Subsystems/Turret/At Target", isAtTarget());
        Logger.recordOutput("Subsystems/Turret/Current", driveMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput("Subsystems/Turret/Degree Difference",
        Math.abs(getWrappedError(currentDegWrapped, targetDegWrapped)));
    }
}
