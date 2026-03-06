package frc.robot;

import ca.team4308.absolutelib.control.RazerWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.turretSubsystem;

public class RobotContainer {
    private Double targetAngle = 0.0;

    public turretSubsystem m_turret = new turretSubsystem();

    public RazerWrapper driverController = new RazerWrapper(0);

    public RobotContainer() {
        driverController.A.onTrue(new InstantCommand(() -> targetAngle = 180.0));
        driverController.B.onTrue(new InstantCommand(() -> targetAngle = 360.0));
        driverController.X.onTrue(new InstantCommand(() -> targetAngle = 420.0));
        driverController.Y.onTrue(new InstantCommand(() -> targetAngle = 500.0));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void robotPeriodic() {
        targetAngle += driverController.getLeftY();
        m_turret.setTarget(targetAngle);
    }
}