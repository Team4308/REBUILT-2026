package frc.robot;

public final class Constants {

  public static final class Intake {

    // CAN IDs
    public static final int ROLLER_ID = 20;
    public static final int PIVOT_ID = 21;

    // Roller tuning
    public static final double ROLLER_GEAR_RATIO = 1.0;
    public static final double ROLLER_KP = 0.12;
    public static final double ROLLER_KI = 0.0;
    public static final double ROLLER_KD = 0.0;
    public static final double ROLLER_KV = 0.12;

    public static final double ROLLER_INTAKE_RPM = 4500.0;

    // Pivot geometry
    public static final double PIVOT_GEAR_RATIO = 100.0;
    public static final double PIVOT_KP = 40.0;
    public static final double PIVOT_KI = 0.0;
    public static final double PIVOT_KD = 2.0;
    public static final double PIVOT_KG = 0.35; // gravity feedforward
    public static final double PIVOT_KV = 1.2;
    public static final double PIVOT_KA = 0.08;

    // Motion Magic
    public static final double MAX_VEL_DEG_PER_SEC = 300.0;
    public static final double MAX_ACCEL_DEG_PER_SEC2 = 600.0;

    // Angles
    public static final double RETRACTED_ANGLE_DEG = 0.0;
    public static final double INTAKE_ANGLE_DEG = 65.0;

    // Agitate
    public static final double AGITATE_LOW_DEG = 45.0;
    public static final double AGITATE_HIGH_DEG = 70.0;
    public static final double ANGLE_TOLERANCE_DEG = 1.5;
  }
}


