package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX m_hoodMotor = new TalonFX(Constants.Hood.HoodMotor);

    private final double TOTAL_GEAR_RATIO = 50.0 * (42.0 / 12.0); 
    private final double FORWARD_SOFT_LIMIT_ANGLE = 120.0;
    private final double REVERSE_SOFT_LIMIT_ANGLE = 0.0;
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0.28, 0.0155, 0.01);
    private final ProfiledPIDController pidController = new ProfiledPIDController(
        0.06, 0.0, 0.0, 
        new TrapezoidProfile.Constraints(500, 1000)
    );

    private double targetAngle = 0;
    private String currentState = "Idle";
    private boolean isStateManaged = false;
    public HoodSubsystem() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Software Limits
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (FORWARD_SOFT_LIMIT_ANGLE / 360.0) * TOTAL_GEAR_RATIO;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (REVERSE_SOFT_LIMIT_ANGLE / 360.0) * TOTAL_GEAR_RATIO;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        m_hoodMotor.getConfigurator().apply(talonFXConfigs);
        m_hoodMotor.setPosition(0);
    }

    public double getHoodAngle() {
        return (m_hoodMotor.getPosition().getValueAsDouble() / TOTAL_GEAR_RATIO) * 360.0;
    }

    public void setHoodAngle(double angle) {
        this.targetAngle = Math.min(Math.max(angle, REVERSE_SOFT_LIMIT_ANGLE), FORWARD_SOFT_LIMIT_ANGLE);
    }

    public boolean isAtPosition() {
        // Uses a tolerance value from Constants 
        return Math.abs(getHoodAngle() - targetAngle) < Constants.Hood.kToleranceDegrees;
    }

    // Move to angle (Supplier allows for dynamic targets like Limelight)
    public Command moveHood(Supplier<Double> angleSupplier) {
        return run(() -> setHoodAngle(angleSupplier.get())).until(this::isAtPosition);
    }

    //Move to angle with Timeout
    public Command moveHood(Supplier<Double> angleSupplier, double timeoutSeconds) {
        return moveHood(angleSupplier).withTimeout(timeoutSeconds);
    }

    public void resetHood() {
        if (m_hoodMotor.getSupplyCurrent().getValueAsDouble() < 20.0) { // 20A threshold
            m_hoodMotor.setVoltage(-2.0); 
        } else {
            m_hoodMotor.setVoltage(0);
            m_hoodMotor.setPosition(0);
        }
    }

    public Command resetHoodCommand() {
        return run(this::resetHood).until(() -> m_hoodMotor.getSupplyCurrent().getValueAsDouble() > 20.0).andThen(runOnce(() -> m_hoodMotor.setPosition(0)));
    }

    public void setState(String state) { this.currentState = state; }
    public void setStateBased(boolean using) { this.isStateManaged = using; }

    // Placeholder for Vision/Strategy Logic
    public void aimAtHub() { setHoodAngle( 0 ); }
    public Command aimAtHubCommand() { return run(this::aimAtHub); }

    public void aimAtPassingZone() { setHoodAngle( Constants.Hood.kPassingAngle ); }
    public Command aimAtPassingZoneCommand() { return run(this::aimAtPassingZone); }

    @Override
    public void periodic() {
        double currentAngle = getHoodAngle();
        double pidOutput = pidController.calculate(currentAngle, targetAngle);
        double ffVolts = feedforward.calculate(Units.degreesToRadians(currentAngle), pidController.getSetpoint().velocity);

        m_hoodMotor.setVoltage(pidOutput + ffVolts);

        Logger.recordOutput("Subsystems/Hood/TargetAngle", targetAngle);
        Logger.recordOutput("Subsystems/Hood/CurrentAngle", currentAngle);
        Logger.recordOutput("Subsystems/Hood/State", currentState);
    }
}