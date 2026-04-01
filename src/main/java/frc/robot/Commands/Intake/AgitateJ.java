package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;

public class AgitateJ extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private boolean goingHigh = true;

    public AgitateJ(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        goingHigh = true;
        intakeSubsystem.setIntakeAngle(Constants.Intake.AGITATE_HIGH_DEG);
    }

    @Override
    public void execute() {
        if (intakeSubsystem.isAtAngle()) {
            if (goingHigh) {
                intakeSubsystem.setIntakeAngle(Constants.Intake.AGITATE_LOW_DEG);
                goingHigh = false;
            } else {
                intakeSubsystem.setIntakeAngle(Constants.Intake.AGITATE_HIGH_DEG);
                goingHigh = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}