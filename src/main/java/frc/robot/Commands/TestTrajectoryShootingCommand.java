package frc.robot.Commands;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.HoodSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Util.TrajectoryCalculations;

public class TestTrajectoryShootingCommand extends Command {
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;
    TrajectoryCalculations trajCalc;
    HoodSubsystem hoodSubsystem;

    public TestTrajectoryShootingCommand(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, Supplier<Pose2d> botPose, TrajectoryCalculations trajCalc, HoodSubsystem hoodSubsystem) {
        addRequirements(shooterSubsystem, turretSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.trajCalc = trajCalc;
        this.hoodSubsystem = hoodSubsystem;
    }


    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
            double targetRPM = trajCalc.getNeededRPM();
            double targetTurretAngle = trajCalc.getNeededYaw();
            double targetHoodAngle = trajCalc.getNeededPitch();
            turretSubsystem.setTarget(targetTurretAngle);
            //shooterSubsystem.setTargetSpeed(targetRPM);
            //hoodSubsystem.setHoodAngle(targetHoodAngle);

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
