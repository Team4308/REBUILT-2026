package frc.robot;

import java.lang.ModuleLayer.Controller;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class StateManager {
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

    private final XboxController m_primaryController = new XboxController(Constants.State.kPrimaryControllerPort);
    private final XboxController m_secondaryController = new XboxController(Constants.State.kSecondaryControllerPort);
    SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
    private State currentState = State.Home;
    boolean isIntaking = false;
    boolean active = false;
    boolean isShooting = false;
    boolean passing = false;

    public void setState(State state) {
        this.currentState = state;
    }

    public State getState(State state) {
        return state;
    }

    public StateManager(CommandXboxController controller) {
        configurePrimaryBindings(controller);
        configureSecondaryBindings(controller);
    }

    private void configurePrimaryBindings(CommandXboxController controller) {

        controller.a().onTrue(
                new InstantCommand(() -> {
                    isShooting = (isShooting) ? false : true;
                    if (isShooting) {
                        if (m_swerveSubsystem.getFieldLocation() == "AllianceZone") {
                            if (active) {
                                if (isIntaking) {
                                    setState(State.ActiveTeleopAllianceZoneShootingIntaking);
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

        controller.b().onTrue(
                new InstantCommand(() -> setState()));

        controller.x().onTrue(
                new InstantCommand(() -> setState()));
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

    public void periodic() {

        switch (currentState) {

            case ActiveTeleopAllianceZoneResting:
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
