package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;

public class Agitate254 extends SequentialCommandGroup {

    public Agitate254(IntakeSubsystem intakeSubsystem) {
        addCommands(
                new MoveIntakeTimed(intakeSubsystem, Constants.Intake.AGITATE_HIGH_DEG, 4));
    }
}