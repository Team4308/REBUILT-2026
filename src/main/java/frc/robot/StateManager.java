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
        InactiveTeleopOpponentlZonePassingIntaking,
        ClimbPrepareLeft,
        ClimbPrepareRight,
        ClimbedUp,
        Home
    }

    // Initialize current state to Home
    private State currentState = State.Home;

    // Primary and Secondary controller objects
    private final CommandXboxController m_primaryController = new CommandXboxController(Constants.State.kPrimaryControllerPort);
    private final CommandXboxController m_secondaryController = new CommandXboxController(Constants.State.kSecondaryControllerPort);

    // All subsystem objects for StateManager
    IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
    ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    TurretSubsystem m_turretSubsystem = new TurretSubsystem();
    ClimberSubsystem m_climberSubsystem = new ClimberSubsystem  ();
    SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();

    // Togglable booleans for the secondary controller
    boolean isIntaking = false; 
    boolean active = false;
    boolean isShooting = false;
    boolean passing = false;

    // Setter and getter for state
    public void setState(State state) {
        this.currentState = state;
    }

    public State getState(State state) {
        return state;
    }

    // Constructor
    public StateManager(CommandXboxController controller) {

        // Initialize controllers
        configurePrimaryBindings(controller);
        configureSecondaryBindings(controller);
    }

    // Primary controller config (bindings)
    private void configurePrimaryBindings(CommandXboxController controller) {

        // a is the main setstate function
        controller.a().onTrue(
                new InstantCommand(() -> {

                    // toggle for button
                    isShooting = (isShooting) ? false : true;


                    if (isShooting) {
                        if (m_swerveSubsystem.getFieldLocation() == "AllianceZone") {
                            if (active) {
                                if (isIntaking && isShooting) {
                                    setState(State.ActiveTeleopAllianceZoneShootingIntaking);
                                } else if (isIntaking) {
                                    setState(State.ActiveTeleopAllianceZoneIntaking);
                                } else {
                                    setState(State.ActiveTeleopAllianceZoneShooting);
                                }
                            } else {
                                setState(State.InactiveTeleopAllianceZoneIntaking);
                            }

                        } else if (m_swerveSubsystem.getFieldLocation() == "NeutralZone") {
                            if (active) {
                                if (isIntaking && passing) {
                                    setState(State.ActiveTeleopNeutralZonePassingIntaking);
                                } else if (isIntaking) {
                                    setState(State.ActiveTeleopNeutralZoneIntaking);
                                } else if (passing) {
                                    setState(State.ActiveTeleopNeutralZonePassing);
                                } else {
                                    setState(State.ActiveTeleopNeutralZoneResting);
                                }
                            } else {
                                if (isIntaking && passing) {
                                    setState(State.InactiveTeleopNeutralZonePassingIntaking);
                                } else {
                                    setState(State.InactiveTeleopNeutralZoneIntaking);
                                }
                            }
                        } else if (m_swerveSubsystem.getFieldLocation() == "OpponentZone") {
                            if (active) {

                                if (isIntaking && passing) {
                                    setState(State.ActiveTeleopOpponentZonePassingIntaking);
                                } else if (isIntaking) {
                                    setState(State.ActiveTeleopOpponentZoneIntaking);
                                } else if (passing) {
                                    setState(State.ActiveTeleopOpponentZonePassing);
                                } else {
                                    setState(State.ActiveTeleopOpponentZoneResting);
                                }
                            } else {
                                if (isIntaking && passing) {
                                    setState(State.InactiveTeleopOpponentlZonePassingIntaking);
                                } else {
                                    setState(State.InactiveTeleopOpponentZoneIntaking);
                                }
                            }
                        }

                    } else if (active) {
                        if (m_swerveSubsystem.getFieldLocation() == "AllianceZone") {
                            setState(State.ActiveTeleopAllianceZoneResting);
                        } else if (m_swerveSubsystem.getFieldLocation() == "NeutralZone") {
                            setState(State.ActiveTeleopNeutralZoneResting);
                        } else if (m_swerveSubsystem.getFieldLocation() == "OpponentZone") {
                            setState(State.ActiveTeleopOpponentZoneResting);
                        }

                    } else if (!active) {
                        if (m_swerveSubsystem.getFieldLocation() == "AllianceZone") {
                            setState(State.InactiveTeleopAllianceZoneResting);
                        } else if (m_swerveSubsystem.getFieldLocation() == "NeutralZone") {
                            setState(State.InactiveTeleopNeutralZoneResting);
                        } else if (m_swerveSubsystem.getFieldLocation() == "OpponentZone") {
                            setState(State.InactiveTeleopOpponentZoneResting);
                        }
                    }

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

    }

    // the big back code
    public void periodic() {

        //repeatedly configure the controller as it varies for where we are on the field
        configurePrimaryBindings(m_primaryController);

        // Freaky switch statement ahh
        switch (currentState) {

            case ActiveTeleopAllianceZoneResting:
                m_intakeSubsystem.setState(state.INTAKING);
                m_indexerSubsystem 
                m_hoodSubsystem.setState(RobotState.REST);
                m_shooterSubsystem 
                m_turretSubsystem 
                m_climberSubsystem

            case ActiveTeleopAllianceZoneShooting:
            case ActiveTeleopAllianceZoneIntaking:
            case ActiveTeleopAllianceZoneShootingIntaking:
            case ActiveTeleopNeutralZoneResting:
            case ActiveTeleopNeutralZonePassing:
            case ActiveTeleopNeutralZoneIntaking:
            case ActiveTeleopNeutralZonePassingIntaking:
            case ActiveTeleopOpponentZoneResting:
            case ActiveTeleopOpponentZonePassing:
            case ActiveTeleopOpponentZoneIntaking:
            case ActiveTeleopOpponentZonePassingIntaking:
            case InactiveTeleopAllianceZoneResting:
            case InactiveTeleopAllianceZoneIntaking:
            case InactiveTeleopNeutralZoneResting:
            case InactiveTeleopNeutralZoneIntaking:
            case InactiveTeleopNeutralZonePassingIntaking:
            case InactiveTeleopOpponentZoneResting:
            case InactiveTeleopOpponentZoneIntaking:
            case InactiveTeleopOpponentlZonePassingIntaking:
            case ClimbPrepareLeft:
            case ClimbPrepareRight:
            case ClimbedUp:
            case Home:

        }
    }

}
