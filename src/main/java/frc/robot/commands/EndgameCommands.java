

import ca.team4308.absolutelib.control.RazerWrapper;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.climber.Climbersubsystem;



public class EndgameCommands { 
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

 // this will move the climber to the prepared position and retract the intake
 public static Command prepareClimb(Climebersubsystem climber) {
    return Commands.runOnce(
                climber::extendClimb,
                climber
        );

 }

// This will retract the climber to the climb position assuming it is already extended
 public static Command climbCommand(Climbersubsystem climber) {
    return Commands.runOnce(
                climber::climb, 
                climber
        );
 } 

 // reset climb to home position
 public static Command resetClimbCommand(Climbersubsystem climber) {
    return Commands.runOnce(
                climber::retractClimb, 
                climber
        );
 } 

 // extend climb and auto retract after 2 seconds
 public static Command releaseClimb(Climbersubsystem climber) {
        return Commands.sequence(
                Commands.runOnce(climber::extendClimb, climber),
                Commands.waitSeconds(2),
                Commands.runOnce(climber::retractClimb, climber)
        );
    }

// ToggleClimbCommand 
// If the climber is retracted, prepare
// If the climber is prepared, climb
// If the climber is climbed, move it to prepare
// ***didn't find those states in climbersubsystem*** to be solved
 public static Command toggleClimb(Climbersubsystem climber) {
        return Commands.runOnce(() -> {

            switch (climber.getState()) {

                case retracted:
                    climber.extendClimb();
                    climber.setState(Climbersubsystem.ClimbState.prepared);
                    break;

                case prepared:
                    climber.climb();
                    climber.setState(Climbersubsystem.ClimbState.climbed);
                    break;

                case climbed:
                    climber.extendClimb();
                    climber.setState(Climbersubsystem.ClimbState.prepared);
                    break;
            }

        }, climber);
    }

    
// AutoAlignLeftThenClimbCommand // This will retract the intake, move climbers up, then drive to the right/left side, then do the climbing stuff
// You need to use drivebase.drivetopose() for that.

// AutoAlignRightThenClimbCommand 
     public static Command autoAlignRightThenClimb(
            Climbersubsystem climber,
            DriveSubsystem drive,
            Pose2d rightPose
    ) {
        return Commands.sequence(
            // ***To be solved***
                // Commands.runOnce(climber::retractClimb, climber),
                // drive.driveToPose(rightPose),
                // Commands.runOnce(climber::climb, climber)
        );
    }
    
}