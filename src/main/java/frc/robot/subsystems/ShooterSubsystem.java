package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import co m.ctre.phoenix6.signals.NeutralModeValue;
import absolutelib;

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

    public double maxrpm;
    public double rpm;

    public String state;
    public boolean using;

    public ShooterSubsystem() {
        rightMotor = new TalonFX(Constants.Mapping.ShooterMotor.kMotor1);
        leftMotor = new TalonFX(Constants.Mapping.ShooterMotor.kMotor2);

        rightMotorOut = new DutyCycleOut(0);
        leftMotorOut = new DutyCycleOut(0);

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
    void setTargetSpeed(double rpm) {
        this.rpm = rpm;
        rightVelocity.Velocity = rpm * Constants.Shooter.topMultiplier;
        leftVelocity.Velocity = rpm * Constants.Shooter.bottomMultiplier;
        rightMotor.setControl(rightVelocity);
        leftMotor.setControl(leftVelocity);
    }

     boolean isAtTargetSpeed() {
        double rightError = Math.abs(rightMotor.getVelocity() - rightVelocity.Velocity);
        double leftError = Math.abs(leftMotor.getVelocity() - leftVelocity.Velocity);
        return rightError < Constants.Shooter.kRPMTolerance && leftError < Constants.Shooter.kRPMTolerance;
     }

     void stopMotors() {
        setTargetSpeed(0);
     }

    public Command setShooterSpeed(Supplier<Double> rpm) {      
        return Commands.run(
            () -> setTargetSpeed(rpm.get()),
            this
        ).until(this::isAtTargetSpeed);
     } // sets the speed to rpm, and runs until it has reached the target

         public Command setShooterSpeed(Supplier<Double> rpm, double timeoutMs) {
           return Commands.run(
                () -> setTargetSpeed(rpm.get()),
               this
           ).until(() -> isAtTargetSpeed() || Timer.getFPGATimestamp() >= timeoutMs);
     } // same as setshooterspeed but if the timeout runs out first, it will finish anyways

     void setState(String state) {
        this.state = state;
     } // sets the current state

     void setStateBased(boolean using) {
        this.using = using;
     } // turns on/off the state manager


    }





/* Necessary functions
void setTargetSpeed(double rpm) {} // sets the target speed of the motors to rpm DONE

boolean isAtTargetSpeed() {} // returns whether the target speed is within x rpm of the target (x in Constants.java) DONE

Command setShooterSpeed(Supplier<Double> rpm) {} // sets the speed to rpm, and runs until it has reached the target DONE

Command setShooterSpeed(Supplier<Double>, double timeoutMs) {} // same as setshooterspeed but if the timeout runs out first, it will finish anyways DONE

void stopMotors() {} // sets target to 0, and stops motors DONE

void setShooterSpeedHub() {} // sets the shooter’s speed to the correct speed to target to the hub. Ask nicholas for how to do this
Command setShooterSpeedHub() {} // same as previous, but it runs until interrupted.

void setShooterSpeedPass() {} // sets the shooter’s speed to the correct speed to pass to our zone. Specific location will be in strategy
Command setShooterSpeedPass() {} // same as previous, but it runs until interrupted. lingfeng said 50%

void setState(String state) {} // sets the current state 

void setStateBased(boolean using) {} // turns on/off the state manager 
*/