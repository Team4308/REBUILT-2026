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
      public static final double kV = 0.11;
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
    }

    public static final class Turret {
      public static final int DRIVE_MOTOR_ID = 13;
      public static final int CANCODER1_ID = 17;
      public static final int CANCODER2_ID = 18;

      public final static double GEAR_RATIO_1 = (85. / 17.) * (40. / 31.);
      public final static double GEAR_RATIO_2 = (85. / 17.) * (40. / 33.);
      public final static double PERIOD = 1 / Math.abs(GEAR_RATIO_1 - GEAR_RATIO_2);

      public final static double GEAR_RATIO_MOTOR = (12.0 / 50.0) * (10.0 / 85.0);
      public final static double STOPPED_VELOCITY = 0.1;
      public final static double MIN_DEGREES = 90;
      public final static double MAX_DEGREES = 500;
      public final static double FULL_REVOLUTION_DEG = 360;
      public final static double TURRET_TOLERANCE_DEGREES = 3;

      public final static ArmFeedforward feedforward = new ArmFeedforward(0.24, 0, 0.0075, 0.01);

      public final static ProfiledPIDController pidController = new ProfiledPIDController(
          0.035, 0.0, 0.0,
          new TrapezoidProfile.Constraints(1500, 2000));
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
    public static final double PIVOT_GEAR_RATIO = 81;
    public static final double PIVOT_KP = 0.0;
    public static final double PIVOT_KI = 0.0;
    public static final double PIVOT_KD = 0.02;
    public static final double PIVOT_KS = 0.15;
    public static final double PIVOT_KG = 0.15; // gravity feedforward
    public static final double PIVOT_KV = 0.03;
    public static final double PIVOT_KA = 0.0;

    // Motion Magic
    public static final double MAX_VEL_DEG_PER_SEC = 300.0;
    public static final double MAX_ACCEL_DEG_PER_SEC2 = 600.0;

    // Angles
    public static final double RETRACTED_ANGLE_DEG = 127.0;
    public static final double INTAKE_ANGLE_DEG = 0.0;

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