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

package frc.robot.commands;

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
    m_subsystem.stopControllers();
  }

  @Override
  public void execute() {
    double control = this.control.get();//converting to rpm
    m_subsystem.setMotorOutput(control);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopControllers();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
