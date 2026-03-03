package frc.robot.subsystems;

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

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class ShooterSubsystem extends AbsoluteSubsystem {
    public final TalonFX rightMotor;
    public final TalonFX leftMotor;

    private final TalonFXConfiguration rightConfiguration;
    private final TalonFXConfiguration leftConfiguration;

    final VelocityVoltage velocityVoltage;

    public double bottomMultiplier;
    public double topMultiplier;

    public double targetRPM;

    public enum ShooterState {
        IDLE,
        SHOOTING,
        PASSING,
    }

    private ShooterState currentState = ShooterState.IDLE;
    public boolean usingStateBased = false;

    boolean underTrench = NetworkTableInstance.getDefault().getTable("AdvantageKit/RealOutputs")
            .getEntry("Swerve/UnderTrench").getBoolean(false);

    public ShooterSubsystem() {
        rightMotor = new TalonFX(Constants.Shooting.Mapping.ShooterMotor.kMotor1);
        leftMotor = new TalonFX(Constants.Shooting.Mapping.ShooterMotor.kMotor2);

        velocityVoltage = new VelocityVoltage(0);

        rightConfiguration = new TalonFXConfiguration();
        leftConfiguration = new TalonFXConfiguration();

        rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.18;
        slot0Configs.kV = 0.113;
        slot0Configs.kP = 0.04;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.01;

        rightMotor.getConfigurator().apply(rightConfiguration);
        rightMotor.getConfigurator().apply(slot0Configs);

        leftMotor.getConfigurator().apply(leftConfiguration);
        leftMotor.getConfigurator().apply(slot0Configs);

        targetRPM = 0;
    }

    public void setTargetSpeed(double rpm) {
        this.targetRPM = rpm;
        // VelocityVoltage expects rotations per second (RPS). Convert RPM -> RPS.
        velocityVoltage.Velocity = rpm / 60.0;
        rightMotor.setControl(velocityVoltage);
    }

    public boolean isAtTargetSpeed() {
        double rightRpm = rightMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM);
        // Compare measured RPM to the stored target RPM (both in RPM)
        double error = Math.abs(rightRpm - this.targetRPM);
        return error < Constants.Shooting.Shooter.kRPMTolerance;
    }

    public void stopMotors() {
        setTargetSpeed(0);
    }

    public Command setShooterSpeed(Supplier<Double> rpm) {
        return Commands.run(
                () -> setTargetSpeed(rpm.get()),
                this).until(this::isAtTargetSpeed);
    } // sets the speed to rpm, and runs until it has reached the target

    public Command setShooterSpeed(Supplier<Double> rpm, double timeoutMs) {
        return Commands.run(
                () -> setTargetSpeed(rpm.get()),
                this).until(() -> isAtTargetSpeed() || (Timer.getFPGATimestamp() * 1000.0) >= timeoutMs);
    } // same as setshooterspeed but if the timeout runs out first, it will finish
      // anyways

    public void setStateBased(boolean using) {
        this.usingStateBased = using;
    } // turns on/off the state manager

    @Override
    public Sendable log() {
        return builder -> {
            builder.addDoubleProperty("Target RPM", this::getTargetRPM, null);
            builder.addDoubleProperty("Right Motor RPM",
                    () -> rightMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM), null);
            builder.addDoubleProperty("Left Motor RPM",
                    () -> leftMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM), null);
            builder.addBooleanProperty("At Target Speed", this::isAtTargetSpeed, null);
            builder.addStringProperty("State", () -> this.currentState.toString(), null);
        }; // publishes live shooter data
    }

    public void selectProfileSlot(int i) {
        velocityVoltage.Slot = i;
    } // switches PID slot

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getRPM() {
        return rightMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM);
    }

    public ShooterState getState() {
        return currentState;
    }

    public void setState(ShooterState state) {
        this.currentState = state;
    }

    public void setShooterSpeedPass() {
        setTargetSpeed(Constants.Shooting.Shooter.kPassingRPM);
    } // sets the shooter’s speed to the correct speed to pass to our zone. Specific
      // location will be in strategy

    public Command setShooterSpeedPassCommand() {
        return Commands.run(
                () -> setShooterSpeedPass(),
                this);
    } // same as previous, but it runs until interrupted.

    public void setShooterSpeedHub() {
        setTargetSpeed(Constants.Shooting.Shooter.kMaxRPM);
    }

    public Command setShooterSpeedHubCommand() {
        return Commands.run(() -> setShooterSpeedHub(), this);
    }

    @Override
    public void periodic() {
        if (usingStateBased) {
            switch (currentState) {
                case IDLE:
                    stopMotors();
                    break;
                case SHOOTING:
                    setShooterSpeedHub();
                    break;
                case PASSING:
                    setShooterSpeedPass();
                    break;
            }
        }

        Logger.recordOutput("Subsystems/Shooter/CurRPM", getRPM());
        Logger.recordOutput("Subsystems/Shooter/AtTargetSpeed", isAtTargetSpeed());
    }
}