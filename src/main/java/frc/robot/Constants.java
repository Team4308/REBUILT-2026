package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import swervelib.math.Matter;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

public final class Constants {
  public static final double ROBOT_MASS = Units.lbsToKilograms(140); // TODO: update this to match the wieght in
                                                                     // physicalproperties.json
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
      public static final double TOLERANCE = 1;

      public static final PIDConstants PID = new PIDConstants(5, 0, 0);
    }

    public static final class Translation {
      public static final double TOLERANCE = 0.01;

      public static final PIDConstants PID = new PIDConstants(5, 0, 0);
    }

    public static final class PathFinding {
      public static final double PIDTolerance = Units.inchesToMeters(12); // How close the bot is when it switches to
                                                                          // PID or align
      public static final PathConstraints constraints = new PathConstraints(
          3.0, 3.0,
          Units.degreesToRadians(360), Units.degreesToRadians(360));

      public static final double MIN_ALIGNED_TIME = 500; // minimum time it needs to be at alignment for the pathfinding
                                                         // to end
    }
  }

  public static final class Shooting {
    public static final class Hood {
      public static final int HoodMotor = 10; // CAN ID for the hood motor
      public static final double kToleranceDegrees = 2.0; // Tolerance for position control
      public static final double TOTAL_GEAR_RATIO = 97.4;
      public static final double FORWARD_SOFT_LIMIT_ANGLE = 52.5;
      public static final double REVERSE_SOFT_LIMIT_ANGLE = 7.5;
      public static final double ampThreshold = 3;
      public final static ArmFeedforward feedforward = new ArmFeedforward(0.262, 0, 0.027, 0.04);
      public final static ProfiledPIDController pidController = new ProfiledPIDController(
          0.47, 0.0, 0.01,
          new TrapezoidProfile.Constraints(2000, 2000));
    }

    public static class Shooter {
      public static final double kMaxRPM = 6000.0;
      public static final int kRPMTolerance = 100;
      public static final double kPassingRPM = 3000.0;

      public static final int kMotor1 = 11;
      public static final int kMotor2 = 12;

      public static final double kS = 0.18;
      public static final double kV = 0.113;
      public static final double kP = 0.04;
      public static final double kI = 0;
      public static final double kD = 0.01;
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
