package frc.robot;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.signals.RobotEnableValue;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HoodSubsystem.RobotState;

public class StateManager {

    // enum of states
    public enum State {
        ActiveTeleopAllianceZoneResting,
        ActiveTeleopAllianceZoneShooting,
        ActiveTeleopAllianceZoneIntaking,
        ActiveTeleopAllianceZoneShootingIntaking,
        ActiveTeleopNeutralZoneResting,
        ActiveTeleopNeutralZonePassing,
        ActiveTeleopNeutralZoneIntaking,
        ActiveTeleopNeutralZonePassingIntaking,
        ActiveTeleopOpponentZoneResting,
        ActiveTeleopOpponentZonePassing,
        ActiveTeleopOpponentZoneIntaking,
        ActiveTeleopOpponentZonePassingIntaking,
        InactiveTeleopAllianceZoneResting,
        InactiveTeleopAllianceZoneIntaking,
        InactiveTeleopNeutralZoneResting,
        InactiveTeleopNeutralZoneIntaking,
        InactiveTeleopNeutralZonePassingIntaking,
        InactiveTeleopOpponentZoneResting,
        InactiveTeleopOpponentZoneIntaking,
        InactiveTeleopOpponentZonePassingIntaking,
        ClimbPrepareLeft,
        ClimbPrepareRight,
        ClimbedUp,
        Home
    }

    // Initialize current state to Home
    private State currentState = State.Home;

    // Primary and Secondary controller objects
    private final CommandXboxController m_primaryController = new CommandXboxController(
            Constants.State.kPrimaryControllerPort);
    private final CommandXboxController m_secondaryController = new CommandXboxController(
            Constants.State.kSecondaryControllerPort);

    // All subsystem objects for StateManager
    IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
    ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    TurretSubsystem m_turretSubsystem = new TurretSubsystem();
    ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

    // Togglable booleans for the secondary controller
    boolean isIntaking = false;
    boolean active = false;
    boolean isShooting = false;
    boolean passing = false;
    boolean manual = false;
    boolean STOP = false;
    boolean StopSwerve = false;

    // Setter and getter for state
    public void setRobotState(State state) {
        this.currentState = state;
    }

    public State getRobotState() {
        return currentState;
    }

    // Constructor
    public StateManager() {
        configurePrimaryBindings(m_primaryController);
        configureSecondaryBindings(m_secondaryController);
    }

    public void updateState() {

        if (STOP && StopSwerve) {// stops all motors including swerve
            setRobotState(State.Home);
            m_hoodSubsystem.stopMotors();
            m_intakeSubsystem.stopRoller();
            m_climberSubsystem.stopMotors(); // Climber DOES NOT have a stop method
            m_indexerSubsystem.stopMotors();
            m_shooterSubsystem.stopMotors();
            m_turretSubsystem.stopTurret();
            m_swerveSubsystem.stopSwerve(); // Swerve DOES NOT have any stop method
            m_hoodSubsystem.setUsingState(false);
            m_swerveSubsystem.setUsingState(false);
            m_intakeSubsystem.setStateBased(false);
            m_climberSubsystem.setStateBased(false);
            m_indexerSubsystem.setUsingState(false);
            m_shooterSubsystem.setStateBased(false);
            m_turretSubsystem.setStateBased(false); // Turret DOES NOT have a setstatebased, needs to be added.
            return;
        } else if (STOP) { // stop all motors except for swerve, done so that if a bot is cooked while in
            setRobotState(State.Home); // the middle of field we can still drive the bot back to safety
            m_hoodSubsystem.stopMotors();
            m_intakeSubsystem.stopRoller();
            m_climberSubsystem.stopMotors(); // Climber DOES NOT have a stop method
            m_indexerSubsystem.stopMotors();
            m_shooterSubsystem.stopMotors();
            m_turretSubsystem.stopTurret();
            m_hoodSubsystem.setUsingState(false);
            m_swerveSubsystem.setUsingState(false);
            m_intakeSubsystem.setStateBased(false);
            m_climberSubsystem.setStateBased(false);
            m_indexerSubsystem.setUsingState(false);
            m_shooterSubsystem.setStateBased(false);
            m_turretSubsystem.setStateBased(false); // Turret DOES NOT have a setstatebased, needs to be added.
            return;
        } else {

            if (manual) { // disables the usingState boolean in all subsystems
                setRobotState(State.Home);
                m_hoodSubsystem.setUsingState(false);
                m_swerveSubsystem.setUsingState(false);
                m_intakeSubsystem.setStateBased(false);
                m_climberSubsystem.setStateBased(false);
                m_indexerSubsystem.setUsingState(false);
                m_shooterSubsystem.setStateBased(false);
                m_turretSubsystem.setStateBased(false); // Turret DOES NOT have a setstatebased, needs to be added.
                return;
            } else { // if both conditions pass run all of the shooting code
                m_hoodSubsystem.setUsingState(true);
                m_swerveSubsystem.setUsingState(true);
                m_intakeSubsystem.setStateBased(true);
                m_climberSubsystem.setStateBased(true);
                m_indexerSubsystem.setUsingState(true);
                m_shooterSubsystem.setStateBased(true);
                m_turretSubsystem.setStateBased(true); // Turret DOES NOT have a setstatebased, needs to be added.

                if (isShooting) {
                    if (m_swerveSubsystem.getFieldLocation().equals("AllianceZone")) {
                        if (active) {
                            if (isIntaking && isShooting) {
                                setRobotState(State.ActiveTeleopAllianceZoneShootingIntaking);
                            } else if (isIntaking) {
                                setRobotState(State.ActiveTeleopAllianceZoneIntaking);
                            } else {
                                setRobotState(State.ActiveTeleopAllianceZoneShooting);
                            }
                        } else {
                            setRobotState(State.InactiveTeleopAllianceZoneIntaking);
                        }

                    } else if (m_swerveSubsystem.getFieldLocation().equals("NeutralZone")) {
                        if (active) {
                            if (isIntaking && passing) {
                                setRobotState(State.ActiveTeleopNeutralZonePassingIntaking);
                            } else if (isIntaking) {
                                setRobotState(State.ActiveTeleopNeutralZoneIntaking);
                            } else if (passing) {
                                setRobotState(State.ActiveTeleopNeutralZonePassing);
                            } else {
                                setRobotState(State.ActiveTeleopNeutralZoneResting);
                            }
                        } else {
                            if (isIntaking && passing) {
                                setRobotState(State.InactiveTeleopNeutralZonePassingIntaking);
                            } else {
                                setRobotState(State.InactiveTeleopNeutralZoneIntaking);
                            }
                        }
                    } else if (m_swerveSubsystem.getFieldLocation().equals("OpponentZone")) {
                        if (active) {

                            if (isIntaking && passing) {
                                setRobotState(State.ActiveTeleopOpponentZonePassingIntaking);
                            } else if (isIntaking) {
                                setRobotState(State.ActiveTeleopOpponentZoneIntaking);
                            } else if (passing) {
                                setRobotState(State.ActiveTeleopOpponentZonePassing);
                            } else {
                                setRobotState(State.ActiveTeleopOpponentZoneResting);
                            }
                        } else {
                            if (isIntaking && passing) {
                                setRobotState(State.InactiveTeleopOpponentZonePassingIntaking);
                            } else {
                                setRobotState(State.InactiveTeleopOpponentZoneIntaking);
                            }
                        }
                    }

                } else if (active) {
                    if (m_swerveSubsystem.getFieldLocation().equals("AllianceZone")) {
                        setRobotState(State.ActiveTeleopAllianceZoneResting);
                    } else if (m_swerveSubsystem.getFieldLocation().equals("NeutralZone")) {
                        setRobotState(State.ActiveTeleopNeutralZoneResting);
                    } else if (m_swerveSubsystem.getFieldLocation().equals("OpponentZone")) {
                        setRobotState(State.ActiveTeleopOpponentZoneResting);
                    }

                } else if (!active) {
                    if (m_swerveSubsystem.getFieldLocation().equals("AllianceZone")) {
                        setRobotState(State.InactiveTeleopAllianceZoneResting);
                    } else if (m_swerveSubsystem.getFieldLocation().equals("NeutralZone")) {
                        setRobotState(State.InactiveTeleopNeutralZoneResting);
                    } else if (m_swerveSubsystem.getFieldLocation().equals("OpponentZone")) {
                        setRobotState(State.InactiveTeleopOpponentZoneResting);
                    }
                }
            }
        }
    }

    // Primary controller config (bindings)
    private void configurePrimaryBindings(CommandXboxController controller) {
        controller.a().onTrue(
                new InstantCommand(() -> {
                    isShooting = (isShooting) ? false : true;
                }));
    }

    /*
     * Secondary controller bindings, buttons are toggles and determine:
     * - active/inactive
     * - if the hopper is full
     * - Shooting or not
     * - Passing or not
     */

    private void configureSecondaryBindings(CommandXboxController controller) {
        // Intake Toggle
        controller.a().onTrue(
                new InstantCommand(() -> {
                    isIntaking = (isIntaking) ? false : true;
                }));
        // Active/Inactive toggle
        controller.b().onTrue(
                new InstantCommand(() -> {
                    active = (active) ? false : true;
                }));

        // Passing toggle
        controller.x().onTrue(
                new InstantCommand(() -> {
                    passing = (passing) ? false : true;
                }));

        // manual toggle

        controller.y().onTrue(
                new InstantCommand(() -> {
                    manual = (manual) ? false : true;
                }));

        // emergency stop non-swerve motor toggle

        controller.leftTrigger().onTrue(
                new InstantCommand(() -> {
                    STOP = (STOP) ? false : true;
                }));

        // emergency swerve motor toggle
        controller.rightTrigger().onTrue(
                new InstantCommand(() -> {
                    StopSwerve = (StopSwerve) ? false : true;
                }));
    }

    public void periodic() {

        // repeatedly configure the controller as it varies for where we are on the
        // field
        updateState();
        // Freaky switch statement ahh
        switch (currentState) {

            // All resting cases (Active && Inactive) should all do the same thing
            case ActiveTeleopAllianceZoneResting:
            case ActiveTeleopNeutralZoneResting:
            case ActiveTeleopOpponentZoneResting:
            case InactiveTeleopAllianceZoneResting:
            case InactiveTeleopNeutralZoneResting:
            case InactiveTeleopOpponentZoneResting:

                m_intakeSubsystem.setRobotState(state.INTAKING);
                m_hoodSubsystem.setState(RobotState.REST);
                m_indexerSubsystem.setState(State.IDLE);
                m_shooterSubsystem.setState(ShooterState.IDLE);
                m_climberSubsystem.setState(States.NOT_CLIMBING);
                // Turret does not have a state manager, requires hard coded implementation if
                // not added...
                m_turretSubsystem.setState(/* whatever the rest state is */);

                break;
            case ActiveTeleopAllianceZoneShooting:
                m_intakeSubsystem.setRobotState(state.SHOOTING);
                m_hoodSubsystem.setState(RobotState.SHOOT);
                m_indexerSubsystem.setState(State.SHOOTING);
                m_shooterSubsystem.setState(ShooterState.SHOOTING);
                m_climberSubsystem.setState(States.NOT_CLIMBING);
                m_turretSubsystem.setState(/* whatever the shooting state is */);
                break;
            // All of the intaking states (active/inactive) should do the same thing
            case ActiveTeleopAllianceZoneIntaking:
            case ActiveTeleopNeutralZoneIntaking:
            case ActiveTeleopOpponentZoneIntaking:
            case InactiveTeleopAllianceZoneIntaking:
            case InactiveTeleopNeutralZoneIntaking:
            case InactiveTeleopOpponentZoneIntaking:

                m_intakeSubsystem.setRobotState(state.INTAKING);
                m_hoodSubsystem.setState(RobotState.SHOOT);
                m_indexerSubsystem.setState(State.IDLE);
                m_shooterSubsystem.setState(ShooterState.IDLE);
                m_climberSubsystem.setState(States.NOT_CLIMBING);
                m_turretSubsystem.setState(/* whatever the aiming (not shooting) state is */);
                break;
            case ActiveTeleopAllianceZoneShootingIntaking:
                m_intakeSubsystem.setRobotState(state.INTAKING); // does not agitate tho...
                m_hoodSubsystem.setState(RobotState.SHOOT);
                m_indexerSubsystem.setState(State.SHOOTING);
                m_shooterSubsystem.setState(ShooterState.SHOOTING);
                m_climberSubsystem.setState(States.NOT_CLIMBING);
                m_turretSubsystem.setState(/* whatever the shooting state is */);
                break;
            // passing implementation needs to be more clear, this is a general outline,
            // needs work
            case ActiveTeleopNeutralZonePassing:
            case ActiveTeleopOpponentZonePassing:

                m_intakeSubsystem.setRobotState(state.REST); // does not agitate tho...
                m_hoodSubsystem.setState(RobotState.PASS_LEFT); // set to PASS_LEFT as a placeholder, implementation
                                                                // needs to be refined
                m_indexerSubsystem.setState(State.SHOOTING);
                m_shooterSubsystem.setState(ShooterState.PASSING);
                m_climberSubsystem.setState(States.NOT_CLIMBING);
                m_turretSubsystem.setState(/* whatever the passing state is */);
                break;

            // Since we have PassingIntake for inactive why aren't there PassinIntake states
            // for active?
            case ActiveTeleopNeutralZonePassingIntaking:
            case ActiveTeleopOpponentZonePassingIntaking:
            case InactiveTeleopNeutralZonePassingIntaking:
            case InactiveTeleopOpponentZonePassingIntaking:

                m_intakeSubsystem.setRobotState(state.INTAKING); // does not agitate tho...
                m_hoodSubsystem.setState(RobotState.PASS_LEFT); // set to PASS_LEFT as a placeholder, implementation
                                                                // needs to be refined
                m_indexerSubsystem.setState(State.SHOOTING);
                m_shooterSubsystem.setState(ShooterState.PASSING);
                m_climberSubsystem.setState(States.NOT_CLIMBING);
                m_turretSubsystem.setState(/* whatever the passing state is */);
                break;

            // Climbing states don't have any setup logic so these statements are never
            // called, update when more info is added
            case ClimbPrepareLeft:
                m_intakeSubsystem.setRobotState(state.REST);
                m_hoodSubsystem.setState(RobotState.REST);
                m_indexerSubsystem.setState(State.IDLE);
                m_shooterSubsystem.setState(ShooterState.IDLE);
                m_climberSubsystem.setState(States.GOING_TO_CLIMB);
                m_turretSubsystem.setState(/* whatever the rest/idle state is */);
                break;

            case ClimbPrepareRight:
                m_intakeSubsystem.setRobotState(state.REST);
                m_hoodSubsystem.setState(RobotState.REST);
                m_indexerSubsystem.setState(State.IDLE);
                m_shooterSubsystem.setState(ShooterState.IDLE);
                m_climberSubsystem.setState(States.GOING_TO_CLIMB);
                m_turretSubsystem.setState(/* whatever the rest/idle state is */);
                break;

            case ClimbedUp:
                m_intakeSubsystem.setRobotState(state.REST);
                m_hoodSubsystem.setState(RobotState.REST);
                m_indexerSubsystem.setState(State.IDLE);
                m_shooterSubsystem.setState(ShooterState.IDLE);
                m_climberSubsystem.setState(States.IN_CLIMB);
                m_turretSubsystem.setState(/* whatever the rest/idle state is */);
                break;

            // similar to resting states, however intake is retracted
            case Home:
                m_intakeSubsystem.setRobotState(state.REST);
                m_hoodSubsystem.setState(RobotState.REST);
                m_indexerSubsystem.setState(State.IDLE);
                m_shooterSubsystem.setState(ShooterState.IDLE);
                m_climberSubsystem.setState(States.NOT_CLIMBING);
                m_turretSubsystem.setState(/* whatever the resting state is */);
                break;

        }
    }

}

/*
 * State manager syntax for each subsystem if you want to make changes without
 * opening a bunch
 * of github branches:
 * 
 * StateManager:
 * enum: State
 * states: too many, look up for the enum
 * setter: setState()
 * 
 * IntakeSubsystem:
 * enum: state
 * states: REST, SHOOTING, INTAKING
 * setter: setState()
 * 
 * IndexerSubsystem:
 * enum: State
 * states: IDLE, SHOOTING
 * setter: setState()
 * 
 * ShooterSubsystem:
 * enum: ShooterState
 * states: IDLE, SHOOTING, PASSING
 * setter: setState()
 * 
 * TurretSubsystem (needs a state manager):
 * enum: NA
 * states: NA
 * setter: NA
 * 
 * HoodSubsystem:
 * enum: RobotState
 * states: REST, SHOOT, PASS_LEFT, PASS_RIGHT
 * setter: setState()
 * 
 * ClimberSubsystem:
 * enum: States
 * states: NOT_CLIMBING, GOING_TO_CLIMB, IN_CLIMB
 * setter: setState()
 */