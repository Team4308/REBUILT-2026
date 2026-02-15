package frc.robot.commands;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.Constants;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

public class ShooterCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private final Supplier<Double> control;

  public ShooterCommand(ShooterSubsystem subsystem, Supplier<Double> control) {
    m_subsystem = subsystem;
    this.control = control;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.selectProfileSlot(0);
    m_subsystem.stopMotors();
  }

  @Override
  public void execute() {
    double control = this.control.get();//converting to rpm
    m_subsystem.setTargetSpeed(control);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
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