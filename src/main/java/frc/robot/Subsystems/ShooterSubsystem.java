package frc.robot.Subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Util.SubsystemVerbosity;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_rightMotor;
    private final TalonFX m_leftMotor;

    private final TalonFXConfiguration m_rightConfiguration;
    private final TalonFXConfiguration m_leftConfiguration;

    private final VelocityVoltage m_velocityVoltage;

    public double bottomMultiplier;
    public double topMultiplier;

    private double m_targetRPM = 0;

    private final SubsystemVerbosity verbosity;

    public ShooterSubsystem() {
        m_rightMotor = new TalonFX(Ports.Shooting.Shooter.kShooterMotor1);
        m_leftMotor = new TalonFX(Ports.Shooting.Shooter.kShooterMotor2);

        m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

        m_rightConfiguration = new TalonFXConfiguration();
        m_leftConfiguration = new TalonFXConfiguration();

        m_rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        m_leftMotor.setControl(new Follower(m_rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = Constants.Shooting.Shooter.kS;
        slot0Configs.kV = Constants.Shooting.Shooter.kV;
        slot0Configs.kP = Constants.Shooting.Shooter.kP;
        slot0Configs.kI = Constants.Shooting.Shooter.kI;
        slot0Configs.kD = Constants.Shooting.Shooter.kD;

        m_rightMotor.getConfigurator().apply(m_rightConfiguration);
        m_rightMotor.getConfigurator().apply(slot0Configs);

        m_leftMotor.getConfigurator().apply(m_leftConfiguration);
        m_leftMotor.getConfigurator().apply(slot0Configs);

        verbosity = SubsystemVerbosity.HIGH;
    }

    public void setTargetVoltage(double voltage) {
        m_rightMotor.setVoltage(voltage);
    }

    public void setTargetSpeed(double rpm) {
        m_targetRPM = rpm;
        m_rightMotor.setControl(m_velocityVoltage.withVelocity(rpm / 60.0));
    }

    public boolean isAtTargetSpeed() {
        double rightRpm = m_rightMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM);
        double error = Math.abs(rightRpm - m_targetRPM);
        return error < Constants.Shooting.Shooter.kRPMTolerance;
    }

    public void stopMotors() {
        setTargetSpeed(0);
    }

    public Command setShooterSpeed(Supplier<Double> rpm) {
        return Commands.run(
                () -> setTargetSpeed(rpm.get()));
    }

    public Command setShooterSpeed(Supplier<Double> rpm, double timeoutMs) {
        return Commands.run(
                () -> setTargetSpeed(rpm.get()),
                this).until(() -> isAtTargetSpeed() || (Timer.getFPGATimestamp() * 1000.0) >= timeoutMs);
    }

    public void selectProfileSlot(int i) {
        m_velocityVoltage.Slot = i;
    }

    public double getTargetRPM() {
        return m_targetRPM;
    }

    public double getRPM() {
        return m_rightMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM);
    }

    public void setShooterSpeedPass() {
        setTargetSpeed(Constants.Shooting.Shooter.kPassingRPM);
    }

    public Command setShooterSpeedPassCommand() {
        return Commands.run(
                () -> setShooterSpeedPass(),
                this);
    }

    public void setShooterSpeedHub() {
        setTargetSpeed(Constants.Shooting.Shooter.kMaxRPM);
    }

    public Command setShooterSpeedHubCommand() {
        return Commands.run(() -> setShooterSpeedHub(), this);
    }

    @Override
    public void periodic() {
        if (verbosity == SubsystemVerbosity.LOW || verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Shooter/Is At Target Speed?", isAtTargetSpeed());
            Logger.recordOutput("Subsystems/Shooter/Current RPM", getRPM());
            Logger.recordOutput("Subsystems/Shooter/Target RPM", m_targetRPM);
        }

        if (verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Shooter/Right Motor/Applied Voltage",
                    m_rightMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Shooter/Right Motor/Motor Temperature",
                    m_rightMotor.getDeviceTemp().getValueAsDouble());
            Logger.recordOutput("Subsystems/Shooter/Right Motor/Current",
                    m_rightMotor.getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput("Subsystems/Shooter/Right Motor/Velocity",
                    m_rightMotor.getVelocity().getValueAsDouble());

            Logger.recordOutput("Subsystems/Shooter/Left Motor/Applied Voltage",
                    m_leftMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Shooter/Left Motor/Motor Temperature",
                    m_leftMotor.getDeviceTemp().getValueAsDouble());
            Logger.recordOutput("Subsystems/Shooter/Left Motor/Current",
                    m_leftMotor.getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput("Subsystems/Shooter/Left Motor/Velocity", m_leftMotor.getVelocity().getValueAsDouble());

            Logger.recordOutput("Subsystems/Shooter/RPM Error", Math.abs(getRPM() - m_targetRPM));
        }
    }
}