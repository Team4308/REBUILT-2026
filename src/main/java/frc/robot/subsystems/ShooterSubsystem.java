package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import co m.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

public class ShooterSubsystem extends LogSubsystem{
    public final TalonFX rightMotor;
    public final TalonFX leftMotor;

    private final DutyCycleOut rightMotorOut;
    private final DutyCycleOut leftMotorOut;

    private final TalonFXConfiguration rightConfiguration;
    private final TalonFXConfiguration leftConfiguration;

    final VelocityVoltage rightVelocity;
    final VelocityVoltage leftVelocity;

    public double bottomMultiplier;
    public double topMultiplier;

    public double maxSpeed;

    public ShooterSubsystem() {
        rightMotor = new TalonFX(Constants.Mapping.ShooterMotor.kMotor1);
        leftMotor = new TalonFX(Constants.Mapping.ShooterMotor.kMotor2);

        rightMotorOut = new DutyCycleOut(0);
        leftMotorOut = new DutyCycleOut(0);

        rightVelocity = new VelocityVoltage(0);
        leftVelocity = new VelocityVoltage(0);

        rightConfiguration = new TalonFXConfiguration();
        leftConfiguration = new TalonFXConfiguration();

        // ask if this is necessary
        // rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        rightMotor.getConfigurator().apply(rightConfiguration);
        leftMotor.getConfigurator().apply(leftConfiguration);

        maxSpeed = 100;
        topMultiplier = 1;
        bottomMultiplier = 1;

    }
}




/* Necessary functions
void setTargetSpeed(double rpm) {} // sets the target speed of the motors to rpm

boolean isAtTargetSpeed() {} // returns whether the target speed is within x rpm of the target (x in Constants.java)

Command setShooterSpeed(Supplier<Double> rpm) {} // sets the speed to rpm, and runs until it has reached the target

Command setShooterSpeed(Supplier<Double>, double timeoutMs) {} // same as setshooterspeed but if the timeout runs out first, it will finish anyways

void stopMotors() {} // sets target to 0, and stops motors

void setShooterSpeedHub() {} // sets the shooter’s speed to the correct speed to target to the hub. Ask nicholas for how to do this
Command setShooterSpeedHub() {} // same as previous, but it runs until interrupted.

void setShooterSpeedPass() {} // sets the shooter’s speed to the correct speed to pass to our zone. Specific location will be in strategy
Command setShooterSpeedPass() {} // same as previous, but it runs until interrupted.

void setState(String state) {} // sets the current state

void setStateBased(boolean using) {} // turns on/off the state manager
*/