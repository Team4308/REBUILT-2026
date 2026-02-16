package frc.robot.Util;

public class RobotStates {

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
}
