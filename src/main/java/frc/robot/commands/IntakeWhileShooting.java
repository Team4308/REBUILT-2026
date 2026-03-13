package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HoodSubsystem.RobotState;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class IntakeWhileShooting extends CommandBase {
    private final IntakeSubsystem m_IntakeSubsystem;
    private final IndexerSubsystem m_IndexerSubsystem;
    private final HoodSubsystem m_HoodSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;
    private final TurretSubsystem m_TurretSubsystem;
    private final SwerveSubsystem m_SwerveSubsystem;

    public IntakeWhileShooting(
            IntakeSubsystem intakeSubsystem,
            IndexerSubsystem indexerSubsystem,
            HoodSubsystem hoodSubsystem,
            ShooterSubsystem shooterSubsystem,
            TurretSubsystem turretSubsystem,
            SwerveSubsystem swerveSubsystem) {
        this.m_IntakeSubsystem = intakeSubsystem;
        this.m_IndexerSubsystem = indexerSubsystem;
        this.m_HoodSubsystem = hoodSubsystem;
        this.m_ShooterSubsystem = shooterSubsystem;
        this.m_TurretSubsystem = turretSubsystem;
        this.m_SwerveSubsystem = swerveSubsystem;

        addRequirements(
                m_IntakeSubsystem,
                m_IndexerSubsystem,
                m_HoodSubsystem,
                m_ShooterSubsystem,
                m_TurretSubsystem,
                m_SwerveSubsystem);
    }
    @Override
    public void initialize() {
        // After Intake makes their statemanager update the 2 lines below:
        m_IntakeSubsystem.setRollerSpeed(Constants.Intake.ROLLER_INTAKE_RPM);
        m_IntakeSubsystem.setIntakeAngle(Constants.Intake.INTAKE_ANGLE_DEG);

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