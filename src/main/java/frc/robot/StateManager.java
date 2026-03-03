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
    SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
    private State currentState = State.Home;


    public void setState(State state) {
        this.currentState = state;
    }

    public State getState(State state) {
        return state;
    }


    public StateManager(CommandXboxController controller) {
        configureBindings(controller);
    }

    private void configureBindings(CommandXboxController controller) {

        controller.a().onTrue(
            new InstantCommand(() -> {
                if(m_swerveSubsystem.m_swerveSubsystem() == "AllianceZone"){
                    setState(State.ActiveTeleopAllianceZoneShootingIntaking);
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
