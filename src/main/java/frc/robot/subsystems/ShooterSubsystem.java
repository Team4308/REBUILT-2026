package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
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

    final VelocityVoltage rightVelocity;
    final VelocityVoltage leftVelocity;

    public double bottomMultiplier;
    public double topMultiplier;

    public double rpm;

    public String state;
    public boolean using;

    public ShooterSubsystem() {
        rightMotor = new TalonFX(Constants.Mapping.ShooterMotor.kMotor1);
        leftMotor = new TalonFX(Constants.Mapping.ShooterMotor.kMotor2);

        rightVelocity = new VelocityVoltage(0);
        leftVelocity = new VelocityVoltage(0);

        rightConfiguration = new TalonFXConfiguration();
        leftConfiguration = new TalonFXConfiguration();

        state = "none";
        using = false;

        // ask if this is necessary
        // rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightMotor.getConfigurator().apply(rightConfiguration);
        leftMotor.getConfigurator().apply(leftConfiguration);

        rpm = 0;

    }

    public void setTargetSpeed(double rpm) {
        this.rpm = rpm;
        rightVelocity.Velocity = rpm * Constants.Shooter.topMultiplier;
        leftVelocity.Velocity = rpm * Constants.Shooter.bottomMultiplier;
        rightMotor.setControl(rightVelocity);
        leftMotor.setControl(leftVelocity);
    }

    public boolean isAtTargetSpeed() {
        double rightRpm = rightMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM);
        double leftRpm = leftMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM);
        double rightError = Math.abs(rightRpm - rightVelocity.Velocity);
        double leftError = Math.abs(leftRpm - leftVelocity.Velocity);
        return rightError < Constants.Shooter.kRPMTolerance && leftError < Constants.Shooter.kRPMTolerance;
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
                this).until(() -> isAtTargetSpeed() || Timer.getFPGATimestamp() >= timeoutMs);
    } // same as setshooterspeed but if the timeout runs out first, it will finish
      // anyways

    public void setState(String state) {
        this    .state = state;
    } // sets the current state

    public void setStateBased(boolean using) {
        this.using = using;
    } // turns on/off the state manager

    @Override
    public Sendable log() {
        return builder -> {
        builder.addDoubleProperty("Target RPM", this::getRPM, null);
        builder.addDoubleProperty("Right Motor RPM", () -> rightMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM), null);
        builder.addDoubleProperty("Left Motor RPM", () -> leftMotor.getVelocity().getValue().in(edu.wpi.first.units.Units.RPM), null);
        builder.addBooleanProperty("At Target Speed", this::isAtTargetSpeed, null);
        builder.addStringProperty("State", () -> this.state, null);
        }; // publishes live shooter data 
    }

    public void selectProfileSlot(int i) {
        rightVelocity.Slot = i;
        leftVelocity.Slot = i;
    } // switches  PID slot

    public double getRPM() {
        return this.rpm;
    }

    public void setShooterSpeedPass() {
        setTargetSpeed(Constants.Shooter.kPassingRPM);
    } // sets the shooter’s speed to the correct speed to pass to our zone. Specific
      // location will be in strategy

    public Command setShooterSpeedPassCommand() {
        return Commands.run(
                () -> setShooterSpeedPass(),
                this);
    } // same as previous, but it runs until interrupted.

public void setShooterSpeedHub() {
        setTargetSpeed(Constants.Shooter.kMaxRPM);
    }

    public Command setShooterSpeedHubCommand() {
        return Commands.run(() -> setShooterSpeedHub(), this);
    }

}

/*
 * Necessary functions
 * void setTargetSpeed(double rpm) {} // sets the target speed of the motors to
 * rpm DONE
 * 
 * boolean isAtTargetSpeed() {} // returns whether the target speed is within x
 * rpm of the target (x in Constants.java) DONE
 * 
 * Command setShooterSpeed(Supplier<Double> rpm) {} // sets the speed to rpm,
 * and runs until it has reached the target DONE
 * 
 * Command setShooterSpeed(Supplier<Double>, double timeoutMs) {} // same as
 * setshooterspeed but if the timeout runs out first, it will finish anyways
 * DONE
 * 
 * void stopMotors() {} // sets target to 0, and stops motors DONE
 * 
 * void setShooterSpeedHub() {} // sets the shooter’s speed to the correct speed
 * to target to the hub. Ask nicholas for how to do this DONE
 * Command setShooterSpeedHub() {} // same as previous, but it runs until
 * interrupted. DONE
 * 
 * void setShooterSpeedPass() {} // sets the shooter’s speed to the correct
 * speed to pass to our zone. Specific location will be in strategy
 * Command setShooterSpeedPass() {} // same as previous, but it runs until
 * interrupted. lingfeng said 50% DONE
 * 
 * void setState(String state) {} // sets the current state DONE
 * 
 * void setStateBased(boolean using) {} // turns on/off the state manager Done
 * 
 * make function to turn RPM to percent 0-1, krakens have 3000 max rpm.
 */