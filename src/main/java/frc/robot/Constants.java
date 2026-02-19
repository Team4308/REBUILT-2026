package frc.robot;

public final class Constants {
    public static final class VisionConstants {
        // GLOBAL CONFIG
        public static final String EXPERIMENTAL_ROOT = "Experimental";

        // CONFIDENCE CALCULATIONS
        public static final double SINGLE_TAG_STD_DEV = 1.0;
        public static final double MULTI_TAG_STD_DEV = 0.5;
        public static final double MAX_AMBIGUITY = 0.5; 
        
        // SIMULATION PHYSICS
        public static final double SIM_FPS = 30.0;
        public static final double SIM_AVG_LATENCY_MS = 35.0;
        public static final double SIM_LATENCY_STD_DEV_MS = 5.0;
        public static final double SIM_CALIB_ERROR_AVG = 0.25;
        public static final double SIM_CALIB_ERROR_STD_DEV = 0.08;
        public static final int SIM_RES_WIDTH = 960;
        public static final int SIM_RES_HEIGHT = 800;
        public static final double SIM_DIAG_FOV = 100.0;
    }
}