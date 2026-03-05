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
            new TrapezoidProfile.Constraints(Constants.Hood.max_velocity, Constants.Hood.max_accel)
        );
    }

    public static final class TurretSubsystem {
        public static final long MOD1 = 31;
        public static final long MOD2 = 33;
        public static final int DRIVE_MOTOR_ID = 11;
        public static final int CANCODER1_ID = 17;
        public static final int CANCODER2_ID = 18;

        public static final double SAFE_ANGLE = 0.0;

        public static final Pose2d PASSING_ZONE = new Pose2d(0, 0, new Rotation2d());
        public static final double PASSING_SIDE_ANGLE = 45.0;

        public static final Translation3d FIELD_TARGET = new Translation3d(8.27, 4.1, 2.0);

        public static final double CANCODER1_GEAR_RATIO = (85.0 / 17.0) * (40.0 / 31.0);
        public static final double CANCODER2_GEAR_RATIO = (85.0 / 17.0) * (40.0 / 33.0);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 12.0 / 85.0;

        public static final double kP = 0.20;
        public static final double kI = 0.0;
        public static final double kD = 0.005;

        public static final double MAX_VELOCITY_DEG_S = 260 * 4;
        public static final double MAX_ACCEL_DEG_S2 = 310 * 6;

        public static final double kS = 0.05;
        public static final double kV = 0.002;

        public static boolean stateManagerEnabled = true;
        public static final double TURRET_TOLERANCE_DEGREES = 0.5;

        public static final int TICKS_PER_REV = (int) (MOD1 * MOD2);
        public static final double MIN_TICKS = -TICKS_PER_REV * 0.5;
        public static final double MAX_TICKS = TICKS_PER_REV * 0.5;
        public static final double MIN_DEGREES = -0.5 * 360.0;
        public static final double MAX_DEGREES = 0.5 * 360.0;

        public static final double TURRET_MOI = 0.025;
    }
}
