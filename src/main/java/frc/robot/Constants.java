package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;

public final class Constants {
  public static final double ROBOT_MASS = Units.lbsToKilograms(140);
  public static final Matter CHASSIS = new Matter(
      new Translation3d(Units.inchesToMeters(0.4), Units.inchesToMeters(0.4), Units.inchesToMeters(7.2)),
      ROBOT_MASS);
  public static final double LOOP_TIME = 0.13;
  public static final double MAX_SPEED = Units.feetToMeters(15.1);

  public static final double BOT_WIDTH = Units.inchesToMeters(35);

  public static final class VisionConstants {
    // GLOBAL CONFIG
    public static final String EXPERIMENTAL_ROOT = "Experimental";

    // CONFIDENCE CALCULATIONS
    public static final double SINGLE_TAG_STD_DEV = 1.0;
    public static final double MULTI_TAG_STD_DEV = 0.5;
    public static final double MAX_AMBIGUITY = 0.5; 
    
    // SIMULATION PHYSICS
    public static final double SIM_FPS = 30.0;
    public static final double SIM_AVG_LATENCY_MS = 35.0;
    public static final double SIM_LATENCY_STD_DEV_MS = 5.0;
    public static final double SIM_CALIB_ERROR_AVG = 0.25;
    public static final double SIM_CALIB_ERROR_STD_DEV = 0.08;
    public static final int SIM_RES_WIDTH = 960;
    public static final int SIM_RES_HEIGHT = 800;
    public static final double SIM_DIAG_FOV = 100.0;
  }

  public static final class Swerve {
    public static final class Heading {
      public static final double TOLERANCE = 10; // In degrees

      public static final PIDConstants HEADING_PID = new PIDConstants(2, 0, 0);
    }

    public static final class Translation {
      public static final double TOLERANCE = 0.05;

      public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, 0);
    }

    public static final class PathFinding {
      public static final double PIDTolerance = Units.inchesToMeters(10); // How close the bot is when it switches to
                                                                          // PID or align
      public static final PathConstraints constraints = new PathConstraints(
          3.0, 4.0,
          Units.degreesToRadians(540), Units.degreesToRadians(720));

      public static final double MIN_ALIGNED_TIME = 500; // minimum time it needs to be at alignment for the pathfinding
                                                         // to end
    }
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.0;
    public static final double RIGHT_X_DEADBAND = 0.0;
    public static final double TURN_CONSTANT = 6;
  }

  public static class Auton {
    public static final int maxLength = 10;
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
