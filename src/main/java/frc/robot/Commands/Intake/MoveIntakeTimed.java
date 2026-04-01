package frc.robot.Commands.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class MoveIntakeTimed extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double targetAngle;
    private final double durationSeconds;

    private double startAngle;
    private double timer;

    public MoveIntakeTimed(IntakeSubsystem intakeSubsystem, double targetAngle, double durationSeconds) {
        this.intakeSubsystem = intakeSubsystem;
        this.targetAngle = targetAngle;
        this.durationSeconds = durationSeconds;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        startAngle = intakeSubsystem.getIntakeAngle();
        timer = 0;
    }

    @Override
    public void execute() {
        timer += 0.02;
        double t = MathUtil.clamp(timer / durationSeconds, 0, 1);
        double interpolatedAngle = startAngle + t * (targetAngle - startAngle);
        intakeSubsystem.setIntakeAngle(interpolatedAngle);
    }

    @Override
    public boolean isFinished() {
        return timer >= durationSeconds && intakeSubsystem.isAtAngle();
    }
}