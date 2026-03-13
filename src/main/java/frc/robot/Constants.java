package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
  public static final double BUMPER_HEIGHT = Units.inchesToMeters(6);

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
    public static class Shooter {
      public static final double kMaxRPM = 6000.0;
      public static final int kRPMTolerance = 100;
      public static final double kPassingRPM = 3000.0;

      public static final double kS = 0.4;
      public static final double kV = 0.11;
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class Turret {
      public final static double GEAR_RATIO_1 = (85. / 17.) * (40. / 31.);
      public final static double GEAR_RATIO_2 = (85. / 17.) * (40. / 33.);
      public final static double PERIOD = 1 / Math.abs(GEAR_RATIO_1 - GEAR_RATIO_2);

      public final static double GEAR_RATIO_MOTOR = (12.0 / 50.0) * (10.0 / 85.0);
      public final static double STOPPED_VELOCITY = 0.1;
      public final static double MIN_DEGREES = 90;
      public final static double MAX_DEGREES = 500;
      public final static double FULL_REVOLUTION_DEG = 360;
      public final static double TURRET_TOLERANCE_DEGREES = 5;

      public final static ArmFeedforward feedforward = new ArmFeedforward(0.24, 0, 0.0075, 0.01);

      public final static ProfiledPIDController pidController = new ProfiledPIDController(
          0.035, 0.0, 0.0,
          new TrapezoidProfile.Constraints(1500, 2000));
    }

    public static final class Hood {
      public static final double TOLERANCE_DEGREES = 2.0; // Tolerance for position control
      public static final double TOLERANCE_VELOCITY = 0.5;
      public static final double TOTAL_GEAR_RATIO = 97.4;
      public static final double FORWARD_SOFT_LIMIT_ANGLE = 52.5;
      public static final double REVERSE_SOFT_LIMIT_ANGLE = 7.5;
      public static final double AMP_THRESHOLD = 2;
      public final static ArmFeedforward feedforward = new ArmFeedforward(0.2, 0.0, 0.025, 0.0);
      public final static ProfiledPIDController pidController = new ProfiledPIDController(
          0.005, 0.0, 0.0,
          new TrapezoidProfile.Constraints(2000, 2000));
    }

    public static final class TrajectoryCalc {
      // Rate-limiting
      public static final double MIN_SOLVE_INTERVAL_MS = 250.0;
      public static final double DISTANCE_CHANGE_THRESHOLD_M = 0.05;
      public static final double YAW_CHANGE_THRESHOLD_DEG = 1.0;

      // Shooter geometry
      public static final double SHOOTER_HEIGHT_M = 0.5;
      public static final double SHOOTER_OFFSET_X_M = 0.1;
      public static final double SHOOTER_OFFSET_Y_M = 0.1;

      // Target
      public static final double TARGET_RADIUS_M = 0.45;

      // ShooterConfig
      public static final double MIN_RPM = 0.0;
      public static final double MAX_RPM = Shooter.kMaxRPM;
      public static final double RPM_TO_VELOCITY_FACTOR = 0.00532;
      public static final double MIN_DISTANCE_M = 0.5;
      public static final double MAX_DISTANCE_M = 12.0;
      public static final double RPM_FEEDBACK_THRESHOLD = 50.0;
      public static final double RPM_ABORT_THRESHOLD = 500.0;
      public static final double PITCH_CORRECTION_PER_RPM_DEFICIT = 0.005; // Match ExampleShooter (was 0.05)
      public static final double MOVING_COMPENSATION_GAIN = 1.0;
      public static final int MOVING_ITERATIONS = 5;
      public static final double SAFETY_MAX_EXIT_VELOCITY = 30.0; // Match ExampleShooter default

      // FlywheelConfig
      public static final double FLYWHEEL_DIAMETER_IN = 4.0;
      public static final double FLYWHEEL_COMPRESSION_RATIO = 0.10;
      public static final double FLYWHEEL_GEAR_RATIO = 1.0;
      public static final int FLYWHEEL_MOTORS_PER_WHEEL = 2;
    }
  }

  public static final class Intake {
    // Roller tuning
    public static final double ROLLER_GEAR_RATIO = 1.0;
    public static final double ROLLER_KP = 0.12;
    public static final double ROLLER_KI = 0.0;
    public static final double ROLLER_KD = 0.0;
    public static final double ROLLER_KV = 0.12;

    public static final double ROLLER_INTAKE_RPM = 4500.0;

    // Pivot geometry
    public static final double PIVOT_GEAR_RATIO = 81;

    public static final ArmFeedforward feedforward = new ArmFeedforward(0.15, 0.15, 0.03, 0.0);
    public static final ProfiledPIDController pidController = new ProfiledPIDController(1, 0.0, 0.02,
        new TrapezoidProfile.Constraints(300, 600));

    // Angles
    public static final double RETRACTED_ANGLE_DEG = 127.0;
    public static final double INTAKE_ANGLE_DEG = 0.0;

    // Agitate
    public static final double AGITATE_LOW_DEG = 45.0;
    public static final double AGITATE_HIGH_DEG = 70.0;

    // ToleranceW
    public static final double ANGLE_TOLERANCE_DEG = 3;
    public static final double VELOCITY_TOLERANCE = 3;
  }

  public static class Indexer {
    public static double DEFAULT_INDEXER_VELOCITY = 100;
    public static double DEFAULT_HOPPER_VELOCITY = 100;

    public static double BALL_TUNNEL_GEAR_RATIO = 5 / 3;
    public static double HOPPER_GEAR_RATIO = 9 / 1;

    public static double HOPPER_Ks = 0.5;
    public static double HOPPER_Kv = 0.1;
    public static double HOPPER_Kp = 0.3;
    public static double HOPPER_Ki = 0;
    public static double HOPPER_Kd = 0;

    public static double BALL_TUNNEL_Ks = 0.5;
    public static double BALL_TUNNEL_Kv = 0.1;
    public static double BALL_TUNNEL_Kp = 0.3;
    public static double BALL_TUNNEL_Ki = 0;
    public static double BALL_TUNNEL_Kd = 0;

    public static double PASSIVE_INDEXER_VELOCITY = 50;
    public static double PASSIVE_HOPEPR_VELOCITY = 50;
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

  public static final class Leds {
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 60;
    public static final int[] startIndexes = new int[] { 0, 1, 2, 3, 60 }; // Front, Back, Left, Right, UnderGlow
    public static final int[] viewAngles = new int[] { 0, 90, 180, 270 }; // Front, Back, Left, Right (in Degrees from
                                                                          // facing the front of the robot)
  }

  public static final class Simulation {
    public static final int MAX_FUEL = 50;

    public static final class FuelSim {
      public static final double xMin = -0.6096;
      public static final double xMax = -0.4318;
      public static final double yMin = -0.31623;
      public static final double yMax = 0.31623;
    }
  }
}