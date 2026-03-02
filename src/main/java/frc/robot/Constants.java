package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    public static class Shooter {
        public static final double kMaxRPM = 6000.0;
        public static final int kRPMTolerance = 10;
        public static final double kPassingRPM = 3000.0;
    }

    public static class Mapping {
        public static class ShooterMotor {
            // Change ID later
            public static final int kMotor1 = 11;
            public static final int kMotor2 = 12;
            public static final int kRPMTolerance = 10;
        }
    }
}
