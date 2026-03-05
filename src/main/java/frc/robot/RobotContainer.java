package frc.robot;

import ca.team4308.absolutelib.control.RazerWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.turretSubsystem;

public class RobotContainer {

    public turretSubsystem m_turret = new turretSubsystem();

    public RazerWrapper driverController = new RazerWrapper(0);

    public RobotContainer() {
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void robotPeriodic() {
    }
}