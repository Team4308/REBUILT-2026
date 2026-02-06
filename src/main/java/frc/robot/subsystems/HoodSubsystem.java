package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class HoodSubsystem extends SubsystemBase {
    // Hardware
    public final TalonFX m_hoodMotor = new TalonFX(Ports.hood.hoodMotor);

    // Physical Constants - ADJUST THESE FOR YOUR BOT
    private final double TOTAL_GEAR_RATIO = 50.0 * (22.0 / 12.0); 
    private final double FORWARD_SOFT_LIMIT_ANGLE = 120.0; // Degrees
    private final double REVERSE_SOFT_LIMIT_ANGLE = 0.0;   // Degrees

    // Controller Values - TUNE THESE WITH SYSID
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0.28, 0.0155, 0.01);
    private final ProfiledPIDController pidController = new ProfiledPIDController(
        0.06, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(500, 1000)
    );

    public double targetAngle = 0;
    private boolean atPosition = false;

    public HoodSubsystem() {
        var talonFXConfigs = new TalonFXConfiguration();

        // 1. Set to Brake Mode so the hood doesn't fall when disabled
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (FORWARD_SOFT_LIMIT_ANGLE / 360.0) * TOTAL_GEAR_RATIO;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (REVERSE_SOFT_LIMIT_ANGLE / 360.0) * TOTAL_GEAR_RATIO;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // Apply all configs to the Kraken X44
        m_hoodtMotor.getConfigurator().apply(talonFXConfigs);

        // IMPORTANT: The hood must be at its "Zero" physical position when the robot turns on!
        m_hoodMotor.setPosition(0);
    }

    public double getHoodAngle() {
        double motorRotations = m_hoodMotor.getPosition().getValueAsDouble();
        return (motorRotations / TOTAL_GEAR_RATIO) * 360.0;
    }

    public Command moveHoot(double angle) {
    return runOnce(() -> {
        targetAngle = DoubleUtils.clamp(angle, REVERSE_SOFT_LIMIT_ANGLE, FORWARD_SOFT_LIMIT_ANGLE);
        pidController.reset(getHoodAngle());
    }).andThen(run(() -> {})).until(this::atPosition);
        }


    public boolean atPosition() {
        return atPosition;
    }

    @Override
    public void periodic() {
        double currentAngle = getHoodAngle();

        atPosition = Math.abs(currentAngle - targetAngle) < 3;

        double pidOutput = pidController.calculate(currentAngle, targetAngle);

        double feedforwardVolts = feedforward.calculate(
            Units.degreesToRadians(currentAngle), 
            pidController.getSetpoint().velocity
        );

        // Final output
        m_hoodMotor.setVoltage(pidOutput + feedforwardVolts);

        // Logging for AdvantageScope
        Logger.recordOutput("Subsystems/Hood/TargetAngle", targetAngle);
        Logger.recordOutput("Subsystems/Hood/CurrentAngle", currentAngle);
        Logger.recordOutput("Subsystems/Hood/AtPosition", atPosition);
        Logger.recordOutput("Subsystems/Hood/MotorVoltage", m_hoodMotor.getMotorVoltage().getValueAsDouble());
    }
}