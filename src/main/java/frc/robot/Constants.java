package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

  public static class Vision {
    public static class CAMERA_PLACEHOLDER {
      public static Rotation3d ORIENTATION = new Rotation3d(0, Math.toRadians(-30), Math.toRadians(-180));
      public static Translation3d TRANSLATION = new Translation3d(0, 0, 0.5);
      public static Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1, 1, 4);
      public static Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 2);
    }

    public static class CAMERA_PLACEHOLDER2 {
      public static Rotation3d ORIENTATION = new Rotation3d(0, Math.toRadians(-30), 0);
      public static Translation3d TRANSLATION = new Translation3d(0, 0, 0.5);
      public static Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1, 1, 4);
      public static Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 2);
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
