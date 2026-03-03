package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
    public static final double ROBOT_MASS = Units.lbsToKilograms(140);
    public static final Matter CHASSIS = new Matter(
            new Translation3d(Units.inchesToMeters(0.4), Units.inchesToMeters(0.4), Units.inchesToMeters(7.2)),
            ROBOT_MASS);
    public static final double LOOP_TIME = 0.13;
    public static final double MAX_SPEED = Units.feetToMeters(15.1);

    public static final double BOT_WIDTH = Units.inchesToMeters(35);

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
}