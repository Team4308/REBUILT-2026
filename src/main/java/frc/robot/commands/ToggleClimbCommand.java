
package frc.robot.commands;

import frc.robot.subsystems.Climbersubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ToggleClimbCommand {

    // ToggleClimbCommand
    // If the climber is retracted, prepare
    // If the climber is prepared, climb
    // If the climber is climbed, move it to prepare
    // ***didn't find those states in climbersubsystem*** to be solved

    // private final Climbersubsystem m_climber;

 public toggleClimb(Climbersubsystem climber) {
    // this.m_climber = climber;
        return Commands.runOnce(() -> {

            switch (climber.getState()) {

                case retracted:
                    climber.extendClimb();
                    climber.setState(Climbersubsystem.ClimbState.prepared);
                    break;

                case prepared:
                    climber.climb();
                    climber.setState(Climbersubsystem.ClimbState.climbed);
                    break;

                case climbed:
                    climber.extendClimb();
                    climber.setState(Climbersubsystem.ClimbState.prepared);
                    break;
            }

        }, climber);
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