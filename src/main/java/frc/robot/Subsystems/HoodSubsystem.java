package frc.robot.Subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.Util.TrajectoryCalculations;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX m_hoodMotor = new TalonFX(Constants.Shooting.Hood.HoodMotor);

    private double targetAngle = 0;

    private double angleOffset = Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE;

    public enum RobotState {
        REST,
        PASS_RIGHT,
        PASS_LEFT,
        SHOOT,
    }

    public HoodSubsystem() {
        trajectory = new TrajectoryCalculations();
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_hoodMotor.getConfigurator().apply(talonFXConfigs);
        m_hoodMotor.setPosition(0);
    }

    public double getHoodAngle() {
        return angleOffset
                + (m_hoodMotor.getPosition().getValueAsDouble() / Constants.Shooting.Hood.TOTAL_GEAR_RATIO) * 360.0;
    }

    public Command setHoodAngleCommand(double angle) {
        return run(() -> setHoodAngle(angle)).until(this::isAtPosition);
    }

    public void setHoodAngle(double angle) {
        targetAngle = MathUtil.clamp(
                angle,
                Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE,
                Constants.Shooting.Hood.FORWARD_SOFT_LIMIT_ANGLE);
    }

    public boolean isAtPosition() {
        // Uses a tolerance value from Constants
        return Math.abs(getHoodAngle() - targetAngle) < Constants.Shooting.Hood.kToleranceDegrees
                && m_hoodMotor.getVelocity().getValueAsDouble() < Constants.Shooting.Hood.kVelocityTolerance;
    }

    // Move to angle (Supplier allows for dynamic targets like Limelight)
    public Command moveHood(Supplier<Double> angleSupplier) {
        return run(() -> setHoodAngle(angleSupplier.get())).until(this::isAtPosition);
    }

    // Move to angle with Timeout
    public Command moveHood(Supplier<Double> angleSupplier, double timeoutSeconds) {
        return moveHood(angleSupplier).withTimeout(timeoutSeconds);
    }

    public void resetHood() {
        if (m_hoodMotor.getSupplyCurrent().getValueAsDouble() < Constants.Shooting.Hood.ampThreshold) {
            m_hoodMotor.setVoltage(-2.0);
        } else {
            m_hoodMotor.setVoltage(0);
        }
    }

    public Command resetHoodCommand() {

        return run(this::resetHood)
                .until(() -> m_hoodMotor.getSupplyCurrent().getValueAsDouble() > Constants.Shooting.Hood.ampThreshold)
                .andThen(new InstantCommand(() -> setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE)));
    }

    private RobotState currentState = RobotState.REST;

    private boolean usingState = false;

    private final TrajectoryCalculations trajectory;

    public void setState(RobotState state) {
        this.currentState = state;
    }

    public RobotState getState() {
        return currentState;
    }

    public void setUsingState(boolean using) {
        usingState = using;
    }

    public boolean getUsingState() {
        return usingState;
    }

    public void stopMotors() {
        m_hoodMotor.setVoltage(0);
    }

    private Alliance getAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return Alliance.Blue;
        }

        return alliance.get();
    }

    private void restingState() {
        setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE);
    }

    private void passRightState() {
        final Translation3d passRightPose = (getAlliance() == Alliance.Red)
                ? new Translation3d(
                        FieldLayout.kFieldLength - FieldLayout.ShooterTargets.kPASS_RIGHT_POSE.getX(),
                        FieldLayout.kFieldWidth - FieldLayout.ShooterTargets.kPASS_RIGHT_POSE.getY(),
                        FieldLayout.ShooterTargets.kPASS_RIGHT_POSE.getZ())
                : FieldLayout.ShooterTargets.kPASS_RIGHT_POSE;
        trajectory.setTargetSupplier(() -> {
            return passRightPose;
        });
        trajectory.updateShot();
        setHoodAngle(trajectory.getNeededPitch());
    }

    private void passLeftState() {
        final Translation3d passLeftPose = (getAlliance() == Alliance.Red)
                ? new Translation3d(
                        FieldLayout.kFieldLength - FieldLayout.ShooterTargets.kPASS_LEFT_POSE.getX(),
                        FieldLayout.kFieldWidth - FieldLayout.ShooterTargets.kPASS_LEFT_POSE.getY(),
                        FieldLayout.ShooterTargets.kPASS_LEFT_POSE.getZ())
                : FieldLayout.ShooterTargets.kPASS_LEFT_POSE;
        trajectory.setTargetSupplier(() -> {
            return passLeftPose;
        });
        trajectory.updateShot();
        setHoodAngle(trajectory.getNeededPitch());
    }

    private void shootHubState() {
        final Translation3d hubPose = (getAlliance() == Alliance.Red)
                ? new Translation3d(
                        FieldLayout.kFieldLength - FieldLayout.ShooterTargets.kHUB_POSE.getX(),
                        FieldLayout.ShooterTargets.kHUB_POSE.getY(),
                        FieldLayout.ShooterTargets.kHUB_POSE.getZ())
                : FieldLayout.ShooterTargets.kHUB_POSE;
        trajectory.setTargetSupplier(() -> {
            return hubPose;
        });
        trajectory.updateShot();
        setHoodAngle(trajectory.getNeededPitch());
    }

    @Override
    public void periodic() {
        boolean underTrench = NetworkTableInstance.getDefault()
                .getTable("AdvantageKit/RealOutputs")
                .getEntry("Swerve/UnderTrench")
                .getBoolean(false);
        // Safety override: hood must retract under trench
        if (underTrench) {
            restingState();
        } else if (usingState) {
            switch (currentState) {
                case REST:
                    restingState();
                    break;

                case SHOOT:
                    shootHubState();
                    break;

                case PASS_RIGHT:
                    passRightState();
                    break;

                case PASS_LEFT:
                    passLeftState();
                    break;

                default:
                    restingState();
                    break;
            }
        }

        double currentAngle = getHoodAngle();
        double pidOutput = Constants.Shooting.Hood.pidController.calculate(currentAngle, targetAngle);
        double ffVolts = Constants.Shooting.Hood.feedforward.calculate(
                Constants.Shooting.Hood.pidController.getSetpoint().position,
                Constants.Shooting.Hood.pidController.getSetpoint().velocity);
        m_hoodMotor.setVoltage(pidOutput + ffVolts);

        Logger.recordOutput("Subsystems/Hood/IsAtTarget", isAtPosition());
        Logger.recordOutput("Subsystems/Hood/CurrentAngle", currentAngle);
        Logger.recordOutput("Subsystems/Hood/TargetAngle",
                Constants.Shooting.Hood.pidController.getSetpoint().position);
        Logger.recordOutput("Subsystems/Hood/PidOutput", pidOutput);
        Logger.recordOutput("Subsystems/Hood/FfVolts", ffVolts);
    }
}