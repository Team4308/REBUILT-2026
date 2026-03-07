package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

      public static final int kMotor1 = 11;
      public static final int kMotor2 = 12;

      public static final double kS = 0.4;
      public static final double kV = 0.1;
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class Turret {
      public static final long MOD1 = 31;
      public static final long MOD2 = 33;
      public static final int DRIVE_MOTOR_ID = 13;
      public static final int CANCODER1_ID = 17;
      public static final int CANCODER2_ID = 18;

      public static final double SAFE_ANGLE = 0.0;

      public static final Pose2d PASSING_ZONE = new Pose2d(0, 0, new Rotation2d());
      public static final double PASSING_SIDE_ANGLE = 45.0;

      public static final Translation3d FIELD_TARGET = new Translation3d(8.27, 4.1, 2.0);

      // Use only to output degrees based on absolute position, not for calculations,
      // which should use raw encoder values and gear ratios
      // use the oppsite of this to convert from degrees to raw encoder values for
      // calculations like to log and find min max encoder values for unwrapping
      public static final double CANCODER1_GEAR_RATIO = (85.0 / 17.0) * (40.0 / 31.0);
      public static final double CANCODER2_GEAR_RATIO = (85.0 / 17.0) * (40.0 / 33.0);

      // The common numerator of the gear ratios: 85*40/17 = 200.
      // This is the number of "teeth passed" (encoder mesh events) per turret
      // revolution.
      public static final double TEETH_PER_TURRET_REV = 200.0;

      public static final double kP = 0.035;
      public static final double kI = 0.0;
      public static final double kD = 0.0;

      public static final double MAX_VELOCITY_DEG_S = 1500;
      public static final double MAX_ACCEL_DEG_S2 = 3000;

      public static final double kS = 0.24;
      public static final double kV = 0.0078;

      public static boolean stateManagerEnabled = true;
      public static final double TURRET_TOLERANCE_DEGREES = 0.5;
      /** Velocity threshold (deg/s) below which turret is considered "stopped". */
      public static final double VELOCITY_STOPPED_THRESHOLD = 1.0;

      public static final int TICKS_PER_REV = (int) (MOD1 * MOD2);
      public static final double MIN_TICKS = -TICKS_PER_REV * 0.5;
      public static final double MAX_TICKS = TICKS_PER_REV * 0.5;
      public static final double MIN_DEGREES = 90;
      public static final double MAX_DEGREES = 480;

      public static final double TURRET_OFFSET = 1179;

      /** One full turret revolution in degrees. */
      public static final double FULL_REVOLUTION_DEG = 360.0;
      /** Half revolution, used for wrap-around calculations. */
      public static final double HALF_REVOLUTION_DEG = 180.0;
      /** CANcoder wraps at ±0.5 turns; this is the wrap threshold. */
      public static final double CANCODER_WRAP_THRESHOLD = 0.5;
      /** Robot loop period in seconds (20 ms). */
      public static final double LOOP_PERIOD_S = 0.020;
      /** Maximum voltage magnitude sent to the turret motor. */
      public static final double MAX_VOLTAGE = 3.0;
      /** Default encoder-1 offset (overridden by Preferences). */
      public static final double DEFAULT_OFFSET1 = 0.0;
      /** Default encoder-2 offset (overridden by Preferences). */
      public static final double DEFAULT_OFFSET2 = 0.0;
    }

    public static final class Hood {
      public static final int HoodMotor = 10; // CAN ID for the hood motor
      public static final double kToleranceDegrees = 2.0; // Tolerance for position control
      public static final double kVelocityTolerance = 0.5;
      public static final double TOTAL_GEAR_RATIO = 97.4;
      public static final double FORWARD_SOFT_LIMIT_ANGLE = 52.5;
      public static final double REVERSE_SOFT_LIMIT_ANGLE = 7.5;
      public static final double ampThreshold = 2;
      public final static ArmFeedforward feedforward = new ArmFeedforward(0.2, 0.0, 0.025, 0.0);
      public final static ProfiledPIDController pidController = new ProfiledPIDController(
          0.005, 0.0, 0.0,
          new TrapezoidProfile.Constraints(2000, 2000));
    }
  }

  public static final class Intake {

    // CAN IDs
    public static final int ROLLER_ID = 20;
    public static final int PIVOT_ID = 21;

    // Roller tuning
    public static final double ROLLER_GEAR_RATIO = 1.0;
    public static final double ROLLER_KP = 0.12;
    public static final double ROLLER_KI = 0.0;
    public static final double ROLLER_KD = 0.0;
    public static final double ROLLER_KV = 0.12;

    public static final double ROLLER_INTAKE_RPM = 4500.0;

    // Pivot geometry
    public static final double PIVOT_GEAR_RATIO = 100.0;
    public static final double PIVOT_KP = 40.0;
    public static final double PIVOT_KI = 0.0;
    public static final double PIVOT_KD = 2.0;
    public static final double PIVOT_KG = 0.35; // gravity feedforward
    public static final double PIVOT_KV = 1.2;
    public static final double PIVOT_KA = 0.08;

    // Motion Magic
    public static final double MAX_VEL_DEG_PER_SEC = 300.0;
    public static final double MAX_ACCEL_DEG_PER_SEC2 = 600.0;

    // Angles
    public static final double RETRACTED_ANGLE_DEG = 0.0;
    public static final double INTAKE_ANGLE_DEG = 65.0;

    // Agitate
    public static final double AGITATE_LOW_DEG = 45.0;
    public static final double AGITATE_HIGH_DEG = 70.0;

    // Tolerance
    public static final double ANGLE_TOLERANCE_DEG = 1.5;
    public static final double VELOCITY_TOLERANCE = 0.5;
  }

  public static class Indexer {
    public static double IndexerSpeed = 100;
    public static double HopperSpeed = 100;
    public static double IndexerGearRatio = 5 / 3;
    public static double HopperGearRatio = 9 / 1;
    public static int HopperMotorId = 14;
    public static double HopperMotorConfigsKs = 0.5;
    public static double HopperMotorConfigsKv = 0.1;
    public static double HopperMotorConfigsKp = 0.3;
    public static double HopperMotorConfigsKi = 0;
    public static double HopperMotorConfigsKd = 0;

    public static int IndexerMotorId = 15;
    public static double IndexerMotorConfigsKs = 0.5;
    public static double IndexerMotorConfigsKv = 0.1;
    public static double IndexerMotorConfigsKp = 0.3;
    public static double IndexerMotorConfigsKi = 0;
    public static double IndexerMotorConfigsKd = 0;

    public static int BeambreakSensor = 9;
    public static double SlowerIndexerSpeed = 50;
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
}