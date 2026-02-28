package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    public static final class Hood {
        public static final int HoodMotor = 10; // CAN ID for the hood motor
        public static final double kPassingAngle = 30.0; //angle for passing zone
        public static final double kToleranceDegrees = 2.0; // Tolerance for position control
        public static final double TOTAL_GEAR_RATIO = 50.0 * (42.0 / 12.0); // Gear ratio
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
    // Modular gear values for Chinese Remainder Theorem position calculation
    public static final long MOD1 = 31; // Number of gear teeth or positions for encoder 1
    public static final long MOD2 = 33; // Number of gear teeth or positions for encoder 2
    // Motors / Encoders
    public static final int DRIVE_MOTOR_ID = 11; 
    public static final int CANCODER1_ID = 1; 
    public static final int CANCODER2_ID = 2; 

    // Safe angles
    public static final double SAFE_ANGLE = 0.0; // Example safe angle (can be adjusted)

    // Passing 
    public static final Pose2d PASSING_ZONE=  new Pose2d(0,0, new Rotation2d());
    public static final double PASSING_SIDE_ANGLE = 45.0;

    // Gears 
    public static final double CANCODER1_GEAR_RATIO = (85.0 / 17.0) * (40.0 / 31.0);
    public static final double CANCODER2_GEAR_RATIO = (85.0 / 17.0) * (40.0 / 33.0);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 12.0 / 85.0;


        // PID & FF
    public static final ProfiledPIDController pidController = new ProfiledPIDController(
            0.05, 0.0, 0.0,
            new TrapezoidProfile.Constraints(500, 1000)
    );
    public static  final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.00);
     
    public static boolean stateManagerEnabled = true;
    public static final double TURRET_TOLERANCE_DEGREES = 0.5;

    }
}