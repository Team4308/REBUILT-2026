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

  public boolean isAtTargetSpeed() {
    return m_subsystem.isAtTargetSpeed();
}

  public Command setShooterSpeed(Supplier<Double> rpm) {
    return m_subsystem.setShooterSpeed(rpm);
  } // sets the speed to rpm, and runs until it has reached the target

     public Command setShooterSpeed(Supplier<Double> rpm, double timeoutMs) {
        return m_subsystem.setShooterSpeed(rpm, timeoutMs);
    }

    public void setShooterSpeedPass() {
    m_subsystem.setShooterSpeedPass();
}
  
    public Command setShooterSpeedPassCommand() {
        return new Command() {
            { addRequirements(m_subsystem); }

            @Override
            public void initialize() {
                m_subsystem.setShooterSpeedPass();
            }

            @Override
            public void execute() {
                m_subsystem.setShooterSpeedPass();
            }

            @Override
            public void end(boolean interrupted) {
                m_subsystem.stopMotors();
            }

            @Override
            public boolean isFinished() {
                return false; // runs until interrupted
            }

        };
    }

    public void setState(String state) {
        m_subsystem.setState(state);
    }

    public void setStateBased(boolean using) {
        m_subsystem.setStateBased(using);
    }

};



/* Necessary functions
void setTargetSpeed(double rpm) {} // sets the target speed of the motors to rpm DONE CALLED IN EXECUTE

boolean isAtTargetSpeed() {} // returns whether the target speed is within x rpm of the target (x in Constants.java)

Command setShooterSpeed(Supplier<Double> rpm) {} // sets the speed to rpm, and runs until it has reached the target DONE

Command setShooterSpeed(Supplier<Double>, double timeoutMs) {} // same as setshooterspeed but if the timeout runs out first, it will finish anyways DONE

void stopMotors() {} // sets target to 0, and stops motors DONE CALLED IN END

void setShooterSpeedHub() {} // sets the shooter’s speed to the correct speed to target to the hub. Ask nicholas for how to do this
Command setShooterSpeedHub() {} // same as previous, but it runs until interrupted.

void setShooterSpeedPass() {} // sets the shooter’s speed to the correct speed to pass to our zone. Specific location will be in strategy DONE
Command setShooterSpeedPass() {} // same as previous, but it runs until interrupted. DONE

void setState(String state) {} // sets the current state DONE

void setStateBased(boolean using) {} // turns on/off the state manager DONE
*/