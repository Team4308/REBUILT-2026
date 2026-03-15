package frc.robot.Subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.Util.SubsystemVerbosity;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX m_hoodMotor = new TalonFX(Ports.Shooting.Hood.kHoodId);

    private double targetAngle = 7.5;

    private double angleOffset = Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE;

    private SubsystemVerbosity verbosity;

    private Supplier<Double> turretSupplier;
    private Supplier<Double> simSupplier;
    private double voltage;

    private ProfiledPIDController pidController = Constants.Shooting.Hood.pidController;

    public HoodSubsystem() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_hoodMotor.getConfigurator().apply(talonFXConfigs);
        m_hoodMotor.setPosition(0);

        verbosity = SubsystemVerbosity.HIGH;

        if (Robot.isSimulation()) { // Brute force sim
            pidController.setP(1);
        }
    }

    public double getVoltage() {
        return voltage;
    }

    public double getHoodAngle() {
        if (Robot.isSimulation()) {
            if (simSupplier == null) {
                return 0;
            }
            return simSupplier.get();
        }
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
        return Math.abs(getHoodAngle() - targetAngle) < Constants.Shooting.Hood.TOLERANCE_DEGREES
                && m_hoodMotor.getVelocity().getValueAsDouble() < Constants.Shooting.Hood.TOLERANCE_VELOCITY;
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
        if (m_hoodMotor.getSupplyCurrent().getValueAsDouble() < Constants.Shooting.Hood.AMP_THRESHOLD) {
            m_hoodMotor.setVoltage(-2.0);
        } else {
            m_hoodMotor.setVoltage(0);
        }
    }

    public Command resetHoodCommand() {
        return run(this::resetHood)
                .until(() -> m_hoodMotor.getSupplyCurrent().getValueAsDouble() > Constants.Shooting.Hood.AMP_THRESHOLD)
                .andThen(new InstantCommand(() -> setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE)))
                .andThen(new InstantCommand(() -> m_hoodMotor.setPosition(0)));
    }

    public void stopMotors() {
        m_hoodMotor.setVoltage(0);
        targetAngle = getHoodAngle();
    }

    public void setTurretSupplier(Supplier<Double> turretSupplier) {
        this.turretSupplier = turretSupplier;
    }

    /**
     * This is for Sim only
     * 
     * @return
     */
    public void setSimSupplier(Supplier<Double> supplier) {
        simSupplier = supplier;
    }

    private Pose3d getHoodPose() {
        double turretYawRad = Math.toRadians(turretSupplier.get() + 180);
        double offsetX = 0.109474;
        double offsetZ = 0.08255;
        double rotatedX = offsetX * Math.cos(turretYawRad);
        double rotatedY = offsetX * Math.sin(turretYawRad);
        double pitchRad = Math.toRadians(getHoodAngle() - 7.5);
        return new Pose3d(
                0.1362075 + rotatedX, rotatedY, 0.3370134992 + offsetZ,
                new Rotation3d(0, pitchRad, turretYawRad));
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

        double currentAngle = getHoodAngle();
        double pidOutput = pidController.calculate(currentAngle, targetAngle);
        double ffVolts = Constants.Shooting.Hood.feedforward.calculate(
                pidController.getSetpoint().position,
                pidController.getSetpoint().velocity);
        voltage = pidOutput + ffVolts;
        m_hoodMotor.setVoltage(voltage);

        if (verbosity == SubsystemVerbosity.LOW || verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Hood/Is At Target?", isAtPosition());
            Logger.recordOutput("Subsystems/Hood/Angle", currentAngle);

            Logger.recordOutput("Subsystems/Hood/Pose", getHoodPose());
        }

        if (verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Hood/PidVolts", pidOutput);
            Logger.recordOutput("Subsystems/Hood/FFVolts", ffVolts);
            Logger.recordOutput("Subsystems/Hood/Applied Voltage",
                    m_hoodMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Hood/Motor Temperature",
                    m_hoodMotor.getDeviceTemp().getValueAsDouble());
            Logger.recordOutput("Subsystems/Hood/Current", m_hoodMotor.getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput("Subsystems/Hood/Error",
                    Constants.Shooting.Hood.pidController.getPositionError());
            Logger.recordOutput("Subsystems/Hood/Velocity", m_hoodMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("Subsystems/Hood/Setpoint Angle",
                    Constants.Shooting.Hood.pidController.getSetpoint().position);
            Logger.recordOutput("Subsystems/Hood/Setpoint Velocity",
                    Constants.Shooting.Hood.pidController.getSetpoint().velocity);

        }
    }
}