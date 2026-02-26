package frc.robot.commands;

import frc.robot.subsystems.Climbersubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ReleaseClimbCommand extends Command {

    // extend climb and auto retract after 2 seconds
    // private final Climbersubsystem m_climber;

 public releaseClimb(Climbersubsystem climber) {
    // this.m_climber = climber;
        return Commands.sequence(
                Commands.runOnce(climber::extendClimb, climber),
                Commands.waitSeconds(2),
                Commands.runOnce(climber::retractClimb, climber)
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