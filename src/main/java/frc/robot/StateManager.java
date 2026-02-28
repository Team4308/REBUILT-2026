package frc.robot;

import ca.team4308.absolutelib.math.trajectories.shooter.ShooterSystem;
import ca.team4308.absolutelib.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.turretSubsystem;
import frc.robot.Util.TrajectoryCalculations;
import frc.robot.subsystems.*;

/*
 * READDDD MEEEEE:
 * Most methods and types are incorrect due to them being for other subsystems
 * Make sure to edit to include the proper names and types which match the description
 * Replace with functions for say setting Yaw, setting RPM, and constants in each respective class
 * All the hood methods and constants are correct, no need to change
 */
public class StateManager extends SubsystemBase {

    public enum RobotState {
        ActiveTeleopAllianceZone,
        ActiveTeleopNeutralZone,
        ActiveTeleopOpponentZone,
        InactiveTeleopAllianceZone,
        InactiveTeleopNeutralZone,
        InactiveTeleopOpponentZone,
        EndgameTeleopAllianceZone,
        EndgameTeleopNeutralZone,
        EndgameTeleopOpponentZone,
        Home
    }

    private final HoodSubsystem hood;
    private final ShooterSubsystem shooter;
    private final turretSubsystem turret;
    private final IntakeSubsystem intake;
    private final DriveSubsystem drive;
    private final Arm arm;
    private StateManager.RobotState currentState = StateManager.RobotState.Home;

    private final TrajectoryCalculations trajectory;

    public void setState(StateManager.RobotState state) {
        this.currentState = state;
    }

    public RobotState getState() {
        return currentState;
    }

    public StateManager(
            HoodSubsystem hood,
            ShooterSubsystem shooter,
            turretSubsystem turret,
            IntakeSubsystem intake,
            DriveSubsystem drive
    ) {
        this.hood = hood;
        this.shooter = shooter;
        this.turret = turret;
        this.intake = intake;
        this.drive = drive;

        trajectory = new TrajectoryCalculations();

        trajectory.setPoseSupplier(drive::getPose);
        trajectory.setChassisSupplier(drive::getChassisSpeeds);
        trajectory.setCurrentRPMsupply(shooter::getRPM);
    }

    @Override
    public void periodic() {
        switch (currentState) {

            case Home:
                intake.stopMotors();
                shooter.stopMotors();
                hood.setHoodAngle(Constants.Hood.REVERSE_SOFT_LIMIT_ANGLE);
                break;
            case ActiveTeleopAllianceZone:
                intake.runIntake();
                trajectory.updateShot();
                hood.setHoodAngle(trajectory.getNeededPitch());
                turret.setYaw(trajectory.getNeededYaw());
                shooter.setTargetRPM(trajectory.getNeededRPM());
                break;

            case ActiveTeleopNeutralZone:
                intake.runIntake();
                hood.setHoodAngle(Constants.Hood.kPassingAngle);
                turret.aimAtPassingSide();
                shooter.setTargetRPM(Constants.Shooter.kPassingRPM);
                break;

            case ActiveTeleopOpponentZone:
                intake.runIntake();
                hood.setHoodAngle(Constants.Hood.kPassingAngle);
                turret.aimAtPassingSide();
                shooter.setTargetRPM(Constants.Shooter.kPassingRPM);
                break;

            case InactiveTeleopAllianceZone:
                intake.runIntake();
                shooter.stop();
                hood.setHoodAngle(Constants.Hood.REVERSE_SOFT_LIMIT_ANGLE);
                turret.setYaw(trajectory.getNeededYaw());
                break;

            case InactiveTeleopNeutralZone:
                intake.runIntake();
                hood.setHoodAngle(Constants.Hood.kPassingAngle);
                turret.aimAtPassingSide();
                shooter.setTargetRPM(Constants.Shooter.kPassingRPM);
                break;

            case InactiveTeleopOpponentZone:
                intake.runIntake();
                hood.setHoodAngle(Constants.Hood.kPassingAngle);
                turret.aimAtPassingSide();
                shooter.setTargetRPM(Constants.Shooter.kPassingRPM);
                break;

            case EndgameTeleopAllianceZone:
            case EndgameTeleopNeutralZone:
            case EndgameTeleopOpponentZone:
                intake.stop();
                shooter.stop();
                turret.setSafeAngle();
                hood.setHoodAngle(Constants.Hood.REVERSE_SOFT_LIMIT_ANGLE);
                break;
        }
    }
}
