package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeWhileShooting extends Command {
    IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
   
    @Override
    public void initialize() {
        m_IntakeSubsystem.setRollerSpeed(Constants.Intake.ROLLER_INTAKE_RPM);
        m_IntakeSubsystem.setIntakeAngle(Constants.Intake.INTAKE_ANGLE_DEG);
        m_IndexerSubsystem.setState(IndexerSubsystem.State.SHOOTING);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.stopRoller();
        m_IntakeSubsystem.setIntakeAngle(Constants.Intake.RETRACTED_ANGLE_DEG);
        m_IndexerSubsystem.setState(IndexerSubsystem.State.IDLE);
    }
}