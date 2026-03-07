package frc.robot.Subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.ChineseRemainderSolver;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;

public class TurretSubsystemCRT extends AbsoluteSubsystem {

    private final TalonFX driveMotor;
    private final CANcoder canCoder1;
    private final CANcoder canCoder2;

    private double targetDeg = 0.0;
    private double currentDeg = 0.0;

    private double offset1 = Constants.Shooting.Turret.DEFAULT_OFFSET1;
    private double offset2 = Constants.Shooting.Turret.DEFAULT_OFFSET2;

    // CRT is used only once at boot to establish absolute position.
    // After that we track via encoder-1 deltas (no race conditions).
    private boolean crtLocked = false;
    private double lastRaw1 = 0.0; // previous encoder-1 reading for delta tracking

    // Debug values for logging
    private double dbgRaw1, dbgRaw2;
    private long dbgVal1, dbgVal2, dbgCrtResult;

    private final TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    Constants.Shooting.Turret.MAX_VELOCITY_DEG_S,
                    Constants.Shooting.Turret.MAX_ACCEL_DEG_S2));
    private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State profileGoal = new TrapezoidProfile.State(0, 0);

    private final PIDController pid = new PIDController(
            Constants.Shooting.Turret.kP, Constants.Shooting.Turret.kI, Constants.Shooting.Turret.kD);

    public TurretSubsystemCRT() {
        driveMotor = new TalonFX(Constants.Shooting.Turret.DRIVE_MOTOR_ID);
        canCoder1 = new CANcoder(Constants.Shooting.Turret.CANCODER1_ID);
        canCoder2 = new CANcoder(Constants.Shooting.Turret.CANCODER2_ID);

        // Load persisted offsets if present (stored by calibrateZero()).
        offset1 = Preferences.getDouble("turret.offset1", offset1);
        offset2 = Preferences.getDouble("turret.offset2", offset2);
        System.out.printf("Turret offsets loaded: offset1=%.6f offset2=%.6f%n", offset1, offset2);

        // pid.enableContinuousInput(TurretSubsystem.MIN_DEGREES,
        // TurretSubsystem.MAX_DEGREES);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveConfig);
    }

    public double getAngleWrapped() {
        return inputModulus(getAngle(), Constants.Shooting.Turret.MIN_DEGREES,
                Constants.Shooting.Turret.MAX_DEGREES, Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
    }

    public double getAngle() {
        return currentDeg;
    }

    private void updateCurrentAngle() {
        double raw1 = canCoder1.getAbsolutePosition().getValueAsDouble() - offset1;
        double raw2 = canCoder2.getAbsolutePosition().getValueAsDouble() - offset2;
        Logger.recordOutput("Subsystems/Turret Encoder1", raw1);
        Logger.recordOutput("Subsystems/Turret Encoder2", raw2);

        dbgRaw1 = raw1;
        dbgRaw2 = raw2;

        if (!crtLocked) {

            long val1 = Math.floorMod(Math.round(raw1 * Constants.Shooting.Turret.MOD1),
                    Constants.Shooting.Turret.MOD1);
            long val2 = Math.floorMod(Math.round(raw2 * Constants.Shooting.Turret.MOD2),
                    Constants.Shooting.Turret.MOD2);
            dbgVal1 = val1;
            dbgVal2 = val2;

            long[] result = ChineseRemainderSolver.solvePair(val1, Constants.Shooting.Turret.MOD1, val2,
                    Constants.Shooting.Turret.MOD2);
            if (result == null)
                return;

            long teeth = result[0]; // [0, 1023)
            dbgCrtResult = teeth;

            // Convert to degrees within [0, 360)
            double deg = teeth / Constants.Shooting.Turret.TEETH_PER_TURRET_REV
                    * Constants.Shooting.Turret.FULL_REVOLUTION_DEG;

            Logger.recordOutput("HELP", deg);

            currentDeg = deg;
            targetDeg = deg; // start target at current so turret doesn't jump
            profileSetpoint = new TrapezoidProfile.State(deg, 0);
            profileGoal = new TrapezoidProfile.State(deg, 0);
            lastRaw1 = raw1;
            crtLocked = true;

            return;
        }
        double deltaRaw = raw1 - lastRaw1;

        // Handle CANcoder wrap-around (encoder values are in [0,1) turns)
        if (deltaRaw > Constants.Shooting.Turret.CANCODER_WRAP_THRESHOLD)
            deltaRaw -= 1.0;
        if (deltaRaw < -Constants.Shooting.Turret.CANCODER_WRAP_THRESHOLD)
            deltaRaw += 1.0;

        double deltaDeg = deltaRaw / Constants.Shooting.Turret.CANCODER1_GEAR_RATIO
                * Constants.Shooting.Turret.FULL_REVOLUTION_DEG;
        currentDeg += deltaDeg;
        lastRaw1 = raw1;

    }

    private double inputModulus(double value, double min, double max, double modulus) {
        double wrappedValue = (value - min) % modulus;
        if (wrappedValue < 0)
            wrappedValue += modulus;
        return wrappedValue + min;
    }

    public void setTarget(double degrees) {
        targetDeg = inputModulus(degrees, Constants.Shooting.Turret.MIN_DEGREES,
                Constants.Shooting.Turret.MAX_DEGREES, Constants.Shooting.Turret.FULL_REVOLUTION_DEG);
        profileGoal = new TrapezoidProfile.State(targetDeg, 0);
    }

    public void nudgeTarget(double deltaDeg) {
        targetDeg = MathUtil.clamp(
                targetDeg + deltaDeg, Constants.Shooting.Turret.MIN_DEGREES, Constants.Shooting.Turret.MAX_DEGREES);
        profileGoal = new TrapezoidProfile.State(targetDeg, 0);
    }

    public boolean isAtTarget() {
        return Math.abs(getAngle() - targetDeg) <= Constants.Shooting.Turret.TURRET_TOLERANCE_DEGREES
                && Math.abs(profileSetpoint.velocity) < Constants.Shooting.Turret.VELOCITY_STOPPED_THRESHOLD;
    }

    public Command moveToTarget(Supplier<Double> degrees) {
        return run(() -> setTarget(degrees.get())).until(this::isAtTarget);
    }

    public Command moveToTarget(Supplier<Double> degrees, double timeoutMs) {
        return run(() -> setTarget(degrees.get())).until(this::isAtTarget).withTimeout(timeoutMs / 1000.0);
    }

    public void resetTurret() {
        setTarget(0.0);
    }

    public Command resetTurretCommand() {
        return run(this::resetTurret).until(this::isAtTarget);
    }

    public void aimAtPoint(Translation3d fieldTarget) {
        Pose2d robotPose = robotPoseSupplier.get();
        double dx = fieldTarget.getX() - robotPose.getX();
        double dy = fieldTarget.getY() - robotPose.getY();
        double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = robotPose.getRotation().getDegrees();
        double turretAngleDeg = fieldAngleDeg - robotHeadingDeg;
        setTarget(turretAngleDeg);
    }

    public Command aimAtPointCommand(Translation3d fieldTarget) {
        return run(() -> aimAtPoint(fieldTarget));
    }

    private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

    public void setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.robotPoseSupplier = poseSupplier;
    }

    public void aimAtPassingZone(Pose2d target) {
        Pose2d robotPose = robotPoseSupplier.get();
        double dx = target.getX() - robotPose.getX();
        double dy = target.getY() - robotPose.getY();
        double globalAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeading = robotPose.getRotation().getDegrees();
        setTarget(globalAngleToTarget - robotHeading);
    }

    public void aimAtPassingSide() {
        setTarget(Constants.Shooting.Turret.PASSING_SIDE_ANGLE);
    }

    public Command aimAtPassingZoneCommand(Pose2d target) {
        return run(() -> aimAtPassingZone(target));
    }

    public void setYaw(double angle) {
        setTarget(angle);
    }

    public void setSafeAngle() {
        setTarget(Constants.Shooting.Turret.SAFE_ANGLE);
    }

    public void stopMotors() {
        driveMotor.setVoltage(0);
        profileSetpoint = new TrapezoidProfile.State(getAngle(), 0);
        profileGoal = profileSetpoint;
    }

    @Override
    public void periodic() {
        updateCurrentAngle();

        profileSetpoint = profile.calculate(Constants.Shooting.Turret.LOOP_PERIOD_S, profileSetpoint, profileGoal);

        double pidOutput = pid.calculate(getAngle(), profileSetpoint.position);

        double ffOutput = Constants.Shooting.Turret.kS * Math.signum(profileSetpoint.velocity)
                + Constants.Shooting.Turret.kV * profileSetpoint.velocity;

        double voltage = pidOutput + ffOutput;

        driveMotor.setVoltage(voltage);

        Logger.recordOutput("Subsystems/Turret Angle (wrapped)", getAngleWrapped());
        Logger.recordOutput("Subsystems/Turret Angle (unwrapped)", getAngle());
        Logger.recordOutput("Subsystems/Turret Target (unwrapped)", targetDeg);
        Logger.recordOutput("Subsystems/Turret Target (wrapped)",
                MathUtil.inputModulus(targetDeg, 0, Constants.Shooting.Turret.FULL_REVOLUTION_DEG));
        Logger.recordOutput("Subsystems/Turret Profile Setpoint", profileSetpoint.position);
        Logger.recordOutput("Subsystems/Turret Profile Velocity", profileSetpoint.velocity);
        Logger.recordOutput("Subsystems/Turret PID Output", pidOutput);
        Logger.recordOutput("Subsystems/Turret FF Output", ffOutput);
        Logger.recordOutput("Subsystems/Turret Voltage", voltage);
        Logger.recordOutput("Subsystems/Turret At Target", isAtTarget());
        Logger.recordOutput("Subsystems/Turret Min Limit (deg)", Constants.Shooting.Turret.MIN_DEGREES);
        Logger.recordOutput("Subsystems/Turret Max Limit (deg)", Constants.Shooting.Turret.MAX_DEGREES);
        // CRT debug
        Logger.recordOutput("Subsystems/CRT/Enc1 Raw", dbgRaw1);
        Logger.recordOutput("Subsystems/CRT/Enc2 Raw", dbgRaw2);
        Logger.recordOutput("Subsystems/CRT/Residue1", (double) dbgVal1);
        Logger.recordOutput("Subsystems/CRT/Residue2", (double) dbgVal2);
        Logger.recordOutput("Subsystems/CRT/TeethPassed", (double) dbgCrtResult);
    }

    @Override
    public Sendable log() {
        return null;
    }
}
