package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import frc.robot.Constants;
import frc.robot.Constants.TurretSubsystem;

public class turretSubsystem extends AbsoluteSubsystem {

    private final TalonFX driveMotor;
    private final CANcoder canCoder1;
    private final CANcoder canCoder2;

    private double targetDegUnwrapped = 0.0;
    private double currentDegUnwrapped = 0.0;

    // CRT is used only once at boot to establish absolute position.
    // After that we track via encoder-1 deltas (no race conditions).
    private boolean crtLocked = false;
    private double lastRaw1 = 0.0; // previous encoder-1 reading for delta tracking

    // Debug values for logging
    private double dbgRaw1, dbgRaw2;
    private long dbgVal1, dbgVal2, dbgCrtResult;

    private final TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    TurretSubsystem.MAX_VELOCITY_DEG_S,
                    TurretSubsystem.MAX_ACCEL_DEG_S2
            )
    );
    private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State profileGoal = new TrapezoidProfile.State(0, 0);

    private final PIDController pid = new PIDController(
            TurretSubsystem.kP, TurretSubsystem.kI, TurretSubsystem.kD
    );

    public turretSubsystem() {
        driveMotor = new TalonFX(Constants.TurretSubsystem.DRIVE_MOTOR_ID);
        canCoder1 = new CANcoder(Constants.TurretSubsystem.CANCODER1_ID);
        canCoder2 = new CANcoder(Constants.TurretSubsystem.CANCODER2_ID);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveConfig);

        CANcoderConfiguration ccConfig = new CANcoderConfiguration();
        canCoder1.getConfigurator().apply(ccConfig);
        canCoder2.getConfigurator().apply(ccConfig);
    }

    public double getTurretAngle() {
        return MathUtil.inputModulus(currentDegUnwrapped, 0, 360);
    }

    public double getUnwrappedAngle() {
        return currentDegUnwrapped;
    }

    private void updateCurrentAngle() {
        double raw1 = canCoder1.getAbsolutePosition().getValueAsDouble();
        double raw2 = canCoder2.getAbsolutePosition().getValueAsDouble();

        dbgRaw1 = raw1;
        dbgRaw2 = raw2;

        if (!crtLocked) {

            long val1 = Math.floorMod(Math.round(raw1 * TurretSubsystem.MOD1), TurretSubsystem.MOD1);
            long val2 = Math.floorMod(Math.round(raw2 * TurretSubsystem.MOD2), TurretSubsystem.MOD2);
            dbgVal1 = val1;
            dbgVal2 = val2;

            long[] result = ChineseRemainderSolver.solvePair(val1, TurretSubsystem.MOD1, val2, TurretSubsystem.MOD2);
            if (result == null) return;

            long teeth = result[0]; // [0, 1023)
            dbgCrtResult = teeth;

            // Convert to degrees within [0, 360)
            double deg = teeth / TurretSubsystem.TEETH_PER_TURRET_REV * 360.0;
            deg = MathUtil.inputModulus(deg, 0, 360);

            currentDegUnwrapped = deg;
            lastRaw1 = raw1;
            crtLocked = true;

            System.out.printf("CRT LOCK | res1=%d res2=%d | teeth=%d | deg=%.2f%n",
                    val1, val2, teeth, deg);
            return;
        }
        double deltaRaw = raw1 - lastRaw1;

        if (deltaRaw > 0.5) deltaRaw -= 1.0;
        if (deltaRaw < -0.5) deltaRaw += 1.0;

        double deltaDeg = deltaRaw / TurretSubsystem.CANCODER1_GEAR_RATIO * 360.0;
        currentDegUnwrapped += deltaDeg;
        lastRaw1 = raw1;

        System.out.printf("ENC1 | raw1=%.4f deltaRaw=%.6f | deg=%.2f%n",
                raw1, deltaRaw, currentDegUnwrapped);
    }

    public void setTarget(double degrees) {
        double requestedWrapped = MathUtil.inputModulus(degrees, 0, 360);
        double currentWrapped = MathUtil.inputModulus(currentDegUnwrapped, 0, 360);
        double minDeg = TurretSubsystem.MIN_DEGREES;
        double maxDeg = TurretSubsystem.MAX_DEGREES;

        double delta = requestedWrapped - currentWrapped;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;
        double shortPath = currentDegUnwrapped + delta;

        if (shortPath >= minDeg && shortPath <= maxDeg) {
            targetDegUnwrapped = shortPath;
        } else {
            double longPath = (delta > 0) ? shortPath - 360 : shortPath + 360;
            if (longPath >= minDeg && longPath <= maxDeg) {
                targetDegUnwrapped = longPath;
            } else {
                targetDegUnwrapped = MathUtil.clamp(shortPath, minDeg, maxDeg);
            }
        }

        profileGoal = new TrapezoidProfile.State(targetDegUnwrapped, 0);
    }

    public void nudgeTarget(double deltaDeg) {
        double minDeg = TurretSubsystem.MIN_DEGREES;
        double maxDeg = TurretSubsystem.MAX_DEGREES;
        targetDegUnwrapped = MathUtil.clamp(targetDegUnwrapped + deltaDeg, minDeg, maxDeg);
        profileGoal = new TrapezoidProfile.State(targetDegUnwrapped, 0);
    }

    public boolean isAtTarget() {
        return Math.abs(currentDegUnwrapped - targetDegUnwrapped) <= TurretSubsystem.TURRET_TOLERANCE_DEGREES
                && Math.abs(profileSetpoint.velocity) < 1.0;
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
        setTarget(Constants.TurretSubsystem.PASSING_SIDE_ANGLE);
    }

    public Command aimAtPassingZoneCommand(Pose2d target) {
        return run(() -> aimAtPassingZone(target));
    }

    public void setYaw(double angle) {
        setTarget(angle);
    }

    public void setSafeAngle() {
        setTarget(Constants.TurretSubsystem.SAFE_ANGLE);
    }

    public void stopTurret() {
        driveMotor.setVoltage(0);
        profileSetpoint = new TrapezoidProfile.State(currentDegUnwrapped, 0);
        profileGoal = profileSetpoint;
    }

    @Override
    public void periodic() {
        updateCurrentAngle();

        profileSetpoint = profile.calculate(0.020, profileSetpoint, profileGoal);

        double pidOutput = pid.calculate(currentDegUnwrapped, profileSetpoint.position);

        double ffOutput = TurretSubsystem.kS * Math.signum(profileSetpoint.velocity)
                + TurretSubsystem.kV * profileSetpoint.velocity;

        double voltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
        driveMotor.setVoltage(voltage);

        recordOutput("Turret Angle (wrapped)", getTurretAngle());
        recordOutput("Turret Angle (unwrapped)", currentDegUnwrapped);
        recordOutput("Target (unwrapped)", targetDegUnwrapped);
        recordOutput("Target (wrapped)", MathUtil.inputModulus(targetDegUnwrapped, 0, 360));
        recordOutput("Profile Setpoint", profileSetpoint.position);
        recordOutput("Profile Velocity", profileSetpoint.velocity);
        recordOutput("PID Output", pidOutput);
        recordOutput("FF Output", ffOutput);
        recordOutput("Voltage", voltage);
        recordOutput("At Target", isAtTarget());
        recordOutput("Min Limit (deg)", TurretSubsystem.MIN_DEGREES);
        recordOutput("Max Limit (deg)", TurretSubsystem.MAX_DEGREES);

        // CRT debug
        recordOutput("CRT/Enc1 Raw", dbgRaw1);
        recordOutput("CRT/Enc2 Raw", dbgRaw2);
        recordOutput("CRT/Residue1", (double) dbgVal1);
        recordOutput("CRT/Residue2", (double) dbgVal2);
        recordOutput("CRT/TeethPassed", (double) dbgCrtResult);
    }

    @Override
    public Sendable log() {
        return null;
    }
}
