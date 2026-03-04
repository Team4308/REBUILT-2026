package frc.robot;

import java.lang.ModuleLayer.Controller;

import ca.team4308.absolutelib.control.XBoxWrapper;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class StateManager{
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
    private final XboxController m_StateController = new XboxController(Constants.State.kStateManagerPort);
    private final XboxController m_fuelStatus = new XboxController(2);
    SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
    private State currentState = State.Home;
    boolean isIntaking = false;
    boolean active = false;
    boolean isShooting = false;
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
                if(m_swerveSubsystem.getFieldLocation() == "AllianceZone"){
                    if(active){
                        if(isIntaking){
                            setState(State.ActiveTeleopAllianceZoneShootingIntaking);
                        }
                    }
                    
                }else if(m_swerveSubsystem.getFieldLocation() == "NeutralZone"){
                    setState(State.ActiveTeleopAllianceZoneShootingIntaking);
                }
                else if(m_swerveSubsystem.getFieldLocation() == "OpponentZone"){
                    setState(State.ActiveTeleopOpponentZoneIntaking);
                }
            })
        );

        controller.b().onTrue(
            new InstantCommand(() -> setState())
        );

        controller.x().onTrue(
            new InstantCommand(() -> setState())
        );
    }

    /* Secondary controller bindings, buttons are toggling and determine:
    - active/inactive
    - if the hopper is full
    - Shooting or not */
    private void configureSecondaryBindings(CommandXboxController controller){
        // Intake Toggle
        controller.a().onTrue(
            new InstantCommand(()->{
                isIntaking = (isIntaking) ? false : true;
            })
        );
        // Active/Inactive toggle

        // HUB is active
        controller.x().onTrue(
            new InstantCommand(()->{
                active = (active)? false: true;
            })
        );


    }


    public void periodic(){

        switch(currentState){
            
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
