package frc.robot;

public class Ports {
    public static class Shooting {
        public static class Hood {
            public static final int kHoodId = 41;
        }

        public static class Turret {
            public static final int kTurretMotorId = 40;
        }

        public static class Shooter {
            public static final int kShooterMotor1 = 42;
            public static final int kShooterMotor2 = 43;
        }
    }

    public static class Indexer {
        public static final int kBallTunnelMotorId = 30;
        public static final int kHopperMotor1Id = 31;
        public static final int kHopperMotor2Id = 32;
        public static final int kBeamBreakId = 9;
    }

    public static class Intake {
        public static final int kRollerMotorId = 21;
        public static final int kPivotMotorId = 20;
    }
}
