package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class HoodCommand extends Command {
    private final HoodSubsystem m_HoodSubsystem; 
    private final double m_angle;

    public HoodCommand(HoodSubsystem hood, double angle) {
        this.m_HoodSubsystem = hood;
        this.m_angle = angle;
        
        // Tells the robot that this command uses the Hood
        addRequirements(m_HoodSubsystem);
    }

    @Override
    public void initialize() {
        // Set the target on the subsystem
        m_hood.targetAngle = m_angle;
        
        // We reset the PID controller inside the Subsystem 
        // because the Controller itself is (and should be) private.
        m_hood.resetController(); 
    }

    @Override
    public boolean isFinished() {
        // Returns true when the hood is within the 3-degree tolerance
        return m_hood.atPosition();
    }
}