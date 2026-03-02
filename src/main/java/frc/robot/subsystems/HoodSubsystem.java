package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.TrajectoryCalculations;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX m_hoodMotor = new TalonFX(Constants.Shooting.Hood.HoodMotor);

    private double targetAngle = 0;

    private double angleOffset = Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE;

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

    public void setHoodAngle(double angle) {
        targetAngle = MathUtil.clamp(
                angle,
                Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE,
                Constants.Shooting.Hood.FORWARD_SOFT_LIMIT_ANGLE);
    }

    public boolean isAtPosition() {
        // Uses a tolerance value from Constants
        return Math.abs(getHoodAngle() - targetAngle) < Constants.Shooting.Hood.kToleranceDegrees;
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
                .andThen(new InstantCommand(() -> setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE)))
                .andThen(runOnce(() -> m_hoodMotor.setPosition(0)));
    }

    // States for Hood
    public enum RobotState {
        REST,
        PASS_RIGHT,
        PASS_LEFT,
        SHOOT,
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

    boolean underTrench = false;

    public void stopMotors() {
        m_hoodMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        boolean underTrench = NetworkTableInstance.getDefault()
                .getTable("AdvantageKit/RealOutputs")
                .getEntry("Swerve/UnderTrench")
                .getBoolean(false);
        // Safety override: hood must retract under trench
        if (underTrench) {
            setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE);
        }

        if (usingState) {
            switch (currentState) {

                case REST:
                    setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE);
                    break;

                case SHOOT:// TODO: Make these switch sides based on alliance color
                    trajectory.setTargetSupplier(() -> {
                        return Constants.Shooting.TargetPoses.kHUB_POSE;
                    });
                    trajectory.updateShot();
                    setHoodAngle(trajectory.getNeededPitch());
                    break;

                case PASS_RIGHT:
                    trajectory.setTargetSupplier(() -> {
                        return Constants.Shooting.TargetPoses.kPASS_RIGHT_POSE;
                    });
                    trajectory.updateShot();
                    setHoodAngle(trajectory.getNeededPitch());
                    break;

                case PASS_LEFT:
                    trajectory.setTargetSupplier(() -> {
                        return Constants.Shooting.TargetPoses.kPASS_LEFT_POSE;
                    });
                    trajectory.updateShot();
                    setHoodAngle(trajectory.getNeededPitch());
                    break;

            }
        }

        double currentAngle = getHoodAngle();
        double pidOutput = Constants.Shooting.Hood.pidController.calculate(currentAngle, targetAngle);
        double ffVolts = Constants.Shooting.Hood.feedforward.calculate(
                Units.degreesToRadians(currentAngle),
                Constants.Shooting.Hood.pidController.getSetpoint().velocity);
        double ffFriction = 0.15 * Math.signum(currentAngle - targetAngle);
        m_hoodMotor.setVoltage(pidOutput + ffVolts + ffFriction);

        Logger.recordOutput("Subsystems/Hood/IsAtTarget", isAtPosition());
        Logger.recordOutput("Subsystems/Hood/CurrentAngle", currentAngle);
    }
}