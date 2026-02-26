public class AutoAlignRightThenClimbCommands { 
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

  

 

// ToggleClimbCommand 
// If the climber is retracted, prepare
// If the climber is prepared, climb
// If the climber is climbed, move it to prepare
// ***didn't find those states in climbersubsystem*** to be solved


    
// AutoAlignLeftThenClimbCommand // This will retract the intake, move climbers up, then drive to the right/left side, then do the climbing stuff
// You need to use drivebase.drivetopose() for that.

// AutoAlignRightThenClimbCommand 
     public static Command autoAlignRightThenClimb(
            Climbersubsystem climber,
            DriveSubsystem drive,
            Pose2d rightPose
    ) {
       // return Commands.sequence(
            // ***To be solved***
                // Commands.runOnce(climber::retractClimb, climber),
                // drive.driveToPose(rightPose),
                // Commands.runOnce(climber::climb, climber)
        //);
    }
    
}