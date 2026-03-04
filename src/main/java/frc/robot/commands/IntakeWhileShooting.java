package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HoodSubsystem.RobotState;

public class IntakeWhileShooting extends Command {
    IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
    HoodSubsystem m_HoodSubsystem = new HoodSubsystem();
    ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    TurretSubsystem m_TurretSubsystem = new TurretSubsystem();
    SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();

    @Override
    public void initialize() {
        // After Intake makes their statemanager update the 2 lines below:
        m_IntakeSubsystem.setState(IndexerSubsystem.state.INTAKING);

        if (m_SwerveSubsystem.getFieldLocation.equals("AllianceZone")) {
            m_IndexerSubsystem.setState(IndexerSubsystem.State.SHOOTING);
            m_ShooterSubsystem.setState(ShooterState.SHOOTING);
            m_HoodSubsystem.setState(RobotState.SHOOT);
            m_TurretSubsystem.setState(RobotState.aimAtHub);

        } else if (m_SwerveSubsystem.getFieldLocation.equals("NeutralZone")) {
            m_IndexerSubsystem.setState(IndexerSubsystem.State.SHOOTING);
            m_ShooterSubsystem.setState(ShooterState.PASSING);
            m_TurretSubsystem.setState(RobotState.aimAtPassingZone);


        } else {
            m_IndexerSubsystem.setState(IndexerSubsystem.State.IDLE);
            m_ShooterSubsystem.setState(ShooterState.IDLE);
            m_TurretSubsystem.setState(RobotState.defaultTurret);
            m_HoodSubsystem.setState(RobotState.REST);

        }

    }

    @Override
    public void execute() {

        if (m_SwerveSubsystem.getFieldLocation.equals("AllianceZone")) {
            m_IntakeSubsystem.setState(IndexerSubsystem.state.SHOOTING);
            m_IndexerSubsystem.setState(IndexerSubsystem.State.SHOOTING);
            m_ShooterSubsystem.setState(ShooterState.SHOOTING);
            m_HoodSubsystem.setState(RobotState.SHOOT);
            m_TurretSubsystem.setState(RobotState.aimAtHub);

        } else if (m_SwerveSubsystem.getFieldLocation.equals("NeutralZone")) {
            m_IntakeSubsystem.setState(IndexerSubsystem.state.SHOOTING);
            m_IndexerSubsystem.setState(IndexerSubsystem.State.SHOOTING);
            m_ShooterSubsystem.setState(ShooterState.PASSING);
            m_TurretSubsystem.setState(RobotState.aimAtPassingZone);



        } else {
            m_IndexerSubsystem.setState(IndexerSubsystem.State.IDLE);
            m_ShooterSubsystem.setState(ShooterState.IDLE);
            m_TurretSubsystem.setState(RobotState.defaultTurret);
            m_HoodSubsystem.setState(RobotState.REST);

        }


    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_IntakeSubsystem.setState(IndexerSubsystem.state.REST);
        m_IndexerSubsystem.setState(IndexerSubsystem.State.IDLE);
        m_HoodSubsystem.setState(RobotState.REST);
        m_ShooterSubsystem.setState(ShooterState.IDLE);
        m_TurretSubsystem.setState(RobotState.defaultTurret);

    }
}