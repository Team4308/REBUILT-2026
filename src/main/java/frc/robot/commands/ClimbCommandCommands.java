
package frc.robot.commands;

import frc.robot.subsystems.Climbersubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimbCommandCommands {

    // This will retract the climber to the climb position assuming it is already
    // extended

    // private final Climbersubsystem m_climb;

    public climbCommand(Climbersubsystem climber) {
        // this.m_climber = climber;
        return Commands.runOnce(
                climber::climb, 
                climber
        );
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
