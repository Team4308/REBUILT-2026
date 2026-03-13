package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ShooterSubsystem;

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
    double control = this.control.get();
    m_subsystem.setTargetSpeed(control * 6000);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
};