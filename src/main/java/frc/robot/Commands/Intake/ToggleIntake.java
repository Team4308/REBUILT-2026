package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;

public class ToggleIntake extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double midAngle = (Constants.Intake.INTAKE_ANGLE_DEG + Constants.Intake.RETRACTED_ANGLE_DEG) / 2.0;

    public ToggleIntake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (intakeSubsystem.getIntakeAngle() < midAngle) {
            intakeSubsystem.setIntakeAngle(Constants.Intake.RETRACTED_ANGLE_DEG);
        } else {
            intakeSubsystem.setIntakeAngle(Constants.Intake.INTAKE_ANGLE_DEG);
        }
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isAtAngleStopped();
    }
}