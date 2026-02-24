package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HoodSubsystem.RobotState;

public class IntakeWhileShooting extends Command {
    IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
    HoodSubsystem m_HoodSubsystem = new HoodSubsystem();
    ShooterSubsystem m_ShooterSubsystem = new HoodSubsystem();
    TurretSubsystem m_TurretSubsystem = new TurrentSubsystem();


    @Override
    public void initialize() {
        // After Intake makes their statemanager update the 2 lines below:
        m_IntakeSubsystem.setRollerSpeed(Constants.Intake.ROLLER_INTAKE_RPM);
        m_IntakeSubsystem.setIntakeAngle(Constants.Intake.INTAKE_ANGLE_DEG);
        m_IndexerSubsystem.setState(IndexerSubsystem.State.SHOOTING);
        m_HoodSubsystem.setState(RobotState.SHOOT);
        m_ShooterSubsystem.setState(ShooterState.SHOOTING);
        m_TurretSubsystem.setState(RobotState.aimAtHub); // Has to check whether passing or not, use a button perhaps?
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
        m_HoodSubsystem.setState(RobotState.REST);
        m_ShooterSubsystem.setState(ShooterState.IDLE);
        m_TurretSubsystem.setState(RobotState.defaultTurret);

    }
}