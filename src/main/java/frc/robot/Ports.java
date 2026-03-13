package frc.robot;

public class Ports {
    public static class Shooting {
        public static class Hood {
            public static final int kHoodId = 10;
        }

        public static class Turret {
            public static final int kTurretMotorId = 13;
            public static final int kCanCoder1Id = 17;
            public static final int kCanCoder2Id = 18;
        }

        public static class Shooter {
            public static final int kShooterMotor1 = 11;
            public static final int kShooterMotor2 = 12;
        }
    }

    public static class Indexer {
        public static final int kBallTunnelMotorId = 15;
        public static final int kHopperMotor1Id = 14;
        public static final int kHopperMotor2Id = 25;
        public static final int kBeamBreakId = 9;
    }

    public static class Intake {
        public static final int kRollerMotorId = 20;
        public static final int kPivotMotorId = 21;
    }
}
