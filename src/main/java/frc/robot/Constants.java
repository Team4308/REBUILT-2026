package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    public static final class Shooting {
        public static final class Hood {
            public static final int HoodMotor = 10; // CAN ID for the hood motor
            public static final double kToleranceDegrees = 2.0; // Tolerance for position control
            public static final double TOTAL_GEAR_RATIO = 97.4; // Gear ratio
            public static final double FORWARD_SOFT_LIMIT_ANGLE = 52.5;
            public static final double REVERSE_SOFT_LIMIT_ANGLE = 7.5;
            public static final double ampThreshold = 3;
            public final static ArmFeedforward feedforward = new ArmFeedforward(0.275, 0, 0.015, 0.01);
            public final static ProfiledPIDController pidController = new ProfiledPIDController(
                    0.67, 0.0, 0.0,
                    new TrapezoidProfile.Constraints(1500, 2500));
        }

        public static final class TargetPoses {
            public static final Translation3d kHUB_POSE = new Translation3d(4.0, 0.0, 2.1);
            public static final Translation3d kPASS_RIGHT_POSE = new Translation3d(4.0, 0.5, 2.1);
            public static final Translation3d kPASS_LEFT_POSE = new Translation3d(4.0, -0.5, 2.1);;
        }
    }
}
