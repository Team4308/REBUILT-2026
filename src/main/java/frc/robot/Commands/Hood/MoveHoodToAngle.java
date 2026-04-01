package frc.robot.Commands.Hood;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HoodSubsystem;

public class MoveHoodToAngle extends Command {
    private final HoodSubsystem hoodSubsystem;
    private final double targetAngle;
    private final double timeOut;
    private double startTime;

    public MoveHoodToAngle(HoodSubsystem hoodSubsystem, double targetAngle, double timeOutMs) {
        this.hoodSubsystem = hoodSubsystem;
        this.targetAngle = targetAngle;
        this.timeOut = timeOutMs / 1000.0; // Convert ms to seconds
        addRequirements(hoodSubsystem);
    }

    public MoveHoodToAngle(HoodSubsystem hoodSubsystem, double targetAngle) {
        this(hoodSubsystem, targetAngle, Double.POSITIVE_INFINITY);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        hoodSubsystem.setHoodAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return hoodSubsystem.isAtPosition() ||
                (Timer.getFPGATimestamp() - startTime) > timeOut;
    }

}
