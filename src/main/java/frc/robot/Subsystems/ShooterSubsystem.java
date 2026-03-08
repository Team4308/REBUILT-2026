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
import frc.robot.Util.TrajectoryCalculations;

public class ShooterSubsystem extends SubsystemBase {
    public final TalonFX rightMotor;
    public final TalonFX leftMotor;

    private final TalonFXConfiguration rightConfiguration;
    private final TalonFXConfiguration leftConfiguration;

    final VelocityVoltage velocityVoltage;

    public double bottomMultiplier;
    public double topMultiplier;

    public double targetRPM = 0;

    private TrajectoryCalculations trajectoryCalculations;

    public enum ShooterState {
        IDLE,
        SHOOTING,
        PASSING,
    }

    private ShooterState currentState = ShooterState.IDLE;
    public boolean usingStateBased = false;

    public ShooterSubsystem() {
        rightMotor = new TalonFX(Constants.Shooting.Shooter.kMotor1);
        leftMotor = new TalonFX(Constants.Shooting.Shooter.kMotor2);

        velocityVoltage = new VelocityVoltage(0).withSlot(0);

        rightConfiguration = new TalonFXConfiguration();
        leftConfiguration = new TalonFXConfiguration();

        rightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftMotor.setControl(new Follower(rightMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = Constants.Shooting.Shooter.kS;
        slot0Configs.kV = Constants.Shooting.Shooter.kV;
        slot0Configs.kP = Constants.Shooting.Shooter.kP;
        slot0Configs.kI = Constants.Shooting.Shooter.kI;
        slot0Configs.kD = Constants.Shooting.Shooter.kD;

        rightMotor.getConfigurator().apply(rightConfiguration);
        rightMotor.getConfigurator().apply(slot0Configs);

        leftMotor.getConfigurator().apply(leftConfiguration);
        leftMotor.getConfigurator().apply(slot0Configs);

        trajectoryCalculations = new TrajectoryCalculations();
        trajectoryCalculations.setCurrentRPMsupply(() -> getRPM());
    }

    public void setPoseSupplier(Supplier<Pose2d>) {
        trajectoryCalculations.setChassisSupplier(null);
    }

    public void setTargetVoltage(double voltage) {
        rightMotor.setVoltage(voltage);
    }

    public void setTargetSpeed(double rpm) {
        this.targetRPM = rpm;
        rightMotor.setControl(velocityVoltage.withVelocity(rpm / 60.0));
    }

    public boolean isAtTargetSpeed() {
        double rightRpm = rightMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM);
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
        Logger.recordOutput("Subsystems/Shooter/TargetRPM", targetRPM);
        Logger.recordOutput("Subsystems/Shooter/AtTargetSpeed", isAtTargetSpeed());
    }
}