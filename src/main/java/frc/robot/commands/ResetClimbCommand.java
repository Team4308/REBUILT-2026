
package frc.robot.commands;

import frc.robot.subsystems.Climbersubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ResetClimbCommand extends Command {

    // reset climb to home position

    private final Climbersubsystem m_climber;

 public resetClimbCommand(Climbersubsystem climber) {
    this.m_climber = climber;
    return Commands.runOnce(
                climber::retractClimb, 
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