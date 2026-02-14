package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
}
