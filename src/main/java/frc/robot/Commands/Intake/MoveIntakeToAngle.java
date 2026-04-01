package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class MoveIntakeToAngle extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double targetAngle;
    private final double timeOut;
    private double startTime;

    public MoveIntakeToAngle(IntakeSubsystem intakeSubsystem, double targetAngle, double timeOutMs) {
        this.intakeSubsystem = intakeSubsystem;
        this.targetAngle = targetAngle;
        this.timeOut = timeOutMs / 1000.0; // Convert ms to seconds
        addRequirements(intakeSubsystem);
    }

    public MoveIntakeToAngle(IntakeSubsystem intakeSubsystem, double targetAngle) {
        this(intakeSubsystem, targetAngle, Double.POSITIVE_INFINITY);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        intakeSubsystem.setIntakeAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.isAtAngle() ||
                (Timer.getFPGATimestamp() - startTime) > timeOut;
    }

}
