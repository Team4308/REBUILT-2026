package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    public static final class Hood {
        public static final int HoodMotor = 10;
        public static final double kPassingAngle = 30.0;
        public static final double kToleranceDegrees = 2.0;
        public static final double TOTAL_GEAR_RATIO = 50.0 * (42.0 / 12.0);
        public static final double FORWARD_SOFT_LIMIT_ANGLE = 82.0;
        public static final double REVERSE_SOFT_LIMIT_ANGLE = 47.0;
        public static final double ampThreshold = 20;
        public static final double max_accel = 1000;
        public static final double max_velocity = 500;
        public final static ArmFeedforward feedforward = new ArmFeedforward(0, 0.28, 0.0155, 0.01);
        public final static ProfiledPIDController pidController = new ProfiledPIDController(
                0.06, 0.0, 0.0,
                new TrapezoidProfile.Constraints(Constants.Hood.max_velocity, Constants.Hood.max_accel));
    }

    public static final class TurretSubsystem {
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

        public static final double kP = 0.02;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double MAX_VELOCITY_DEG_S = 260 * 4;
        public static final double MAX_ACCEL_DEG_S2 = 310 * 6;

        public static final double kS = 0.3;
        public static final double kV = 0.01;

        public static boolean stateManagerEnabled = true;
        public static final double TURRET_TOLERANCE_DEGREES = 0.5;
        /** Velocity threshold (deg/s) below which turret is considered "stopped". */
        public static final double VELOCITY_STOPPED_THRESHOLD = 1.0;

        public static final int TICKS_PER_REV = (int) (MOD1 * MOD2);
        public static final double MIN_TICKS = -TICKS_PER_REV * 0.5;
        public static final double MAX_TICKS = TICKS_PER_REV * 0.5;
        public static final double MIN_DEGREES = 180;
        public static final double MAX_DEGREES = 540;

        /** One full turret revolution in degrees. */
        public static final double FULL_REVOLUTION_DEG = 360.0;
        /** Half revolution, used for wrap-around calculations. */
        public static final double HALF_REVOLUTION_DEG = 180.0;
        /** CANcoder wraps at ±0.5 turns; this is the wrap threshold. */
        public static final double CANCODER_WRAP_THRESHOLD = 0.5;
        /** Robot loop period in seconds (20 ms). */
        public static final double LOOP_PERIOD_S = 0.020;
        /** Maximum voltage magnitude sent to the turret motor. */
        public static final double MAX_VOLTAGE = 1.0;
        /** Default encoder-1 offset (overridden by Preferences). */
        public static final double DEFAULT_OFFSET1 = 0.0;
        /** Default encoder-2 offset (overridden by Preferences). */
        public static final double DEFAULT_OFFSET2 = 0.0;
    }
}
