public class ReleaseClimbCommand { 
    /*PrepareClimbCommand // this will move the climber to the prepared position and retract the intake

ClimbCommand // This will retract the climber to the climb position assuming it is already extended

ToggleClimbCommand 
If the climber is retracted, prepare
If the climber is prepared, climb
If the climber is climbed, move it to prepare

resetclimberCommand
Just move the climber to home position

ReleaseClimbCommand // This will move the climber to the top, then wait some amount of time(2 seconds for now), then automatically retract to the bottom.

AutoAlignLeftThenClimbCommand // This will retract the intake, move climbers up, then drive to the right/left side, then do the climbing stuff
You need to use drivebase.drivetopose() for that.

AutoAlignRightThenClimbCommand 
 */

 

 // extend climb and auto retract after 2 seconds
 public static Command releaseClimb(Climbersubsystem climber) {
        return Commands.sequence(
                Commands.runOnce(climber::extendClimb, climber),
                Commands.waitSeconds(2),
                Commands.runOnce(climber::retractClimb, climber)
        );
    }

}