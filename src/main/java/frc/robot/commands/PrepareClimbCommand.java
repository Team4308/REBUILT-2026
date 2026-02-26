package frc.robot.commands;

import frc.robot.subsystems.Climbersubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PrepareClimbCommand extends Command {
    // this will move the climber to the prepared position and retract the intake

    // private final Climbersubsystem m_climber;

 public PrepareClimb(Climebersubsystem climber) {
    // this.m_climber = climber;
    return Commands.runOnce(
                climber::extendClimb,
                climber
        );
 }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {

    }

    @Override
    public void end(boolean interrupted) {
    }
}