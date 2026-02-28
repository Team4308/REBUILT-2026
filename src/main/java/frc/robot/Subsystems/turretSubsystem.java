package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.trajectories.solver.ChineseRemainderSolver;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretSubsystem;
import frc.robot.Util.TrajectoryCalculations;

public class turretSubsystem extends AbsoluteSubsystem {

    private final TalonFX driveMotor;
    private final CANcoder canCoder1;
    private final CANcoder canCoder2;

    private double targetAngle = 0.0;

    public turretSubsystem() {
        driveMotor = new TalonFX(Constants.TurretSubsystem.DRIVE_MOTOR_ID);
        canCoder1 = new CANcoder(Constants.TurretSubsystem.CANCODER1_ID);
        canCoder2 = new CANcoder(Constants.TurretSubsystem.CANCODER2_ID);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveConfig);

        CANcoderConfiguration ccConfig = new CANcoderConfiguration();
        ccConfig.MagnetSensor.MagnetOffset = 0.0;
        canCoder1.getConfigurator().apply(ccConfig);
        canCoder2.getConfigurator().apply(ccConfig);
 
        // Wrap PID controller for continuous input from 0 to 360 degrees
        Constants.TurretSubsystem.pidController.enableContinuousInput(0, 360);
    }

    public double getTurretAngle() {
        double encoder1 = canCoder1.getAbsolutePosition().getValueAsDouble();
        double encoder2 = canCoder2.getAbsolutePosition().getValueAsDouble();
        long val1 = Math.floorMod(Math.round(encoder1 * TurretSubsystem.MOD1), TurretSubsystem.MOD1);
        long val2 = Math.floorMod(Math.round(encoder2 * TurretSubsystem.MOD2), TurretSubsystem.MOD2);

        long[] result = ChineseRemainderSolver.solveTwoCongruences(val1, TurretSubsystem.MOD1, val2,
                TurretSubsystem.MOD2);
        if (result == null) {
            return 0.0;
        }

        double rotations = result[0] / 200.0;
        double degrees = rotations * 360.0;
        return MathUtil.inputModulus(degrees, 0, 360);
    }

    public void setTarget(double degrees) {
        targetAngle = MathUtil.inputModulus(degrees, 0, 360);
    }

    public boolean isAtTarget() {
        return Math.abs(MathUtil.inputModulus(targetAngle - getTurretAngle(), -180,
                180)) <= TurretSubsystem.TURRET_TOLERANCE_DEGREES;
    }

    public Command moveToTarget(Supplier<Double> degrees) {
        return run(() -> {
            setTarget(degrees.get());
        }).until(this::isAtTarget);
    }

    public Command moveToTarget(Supplier<Double> degrees, double timeoutMs) {
        return run(() -> {
            setTarget(degrees.get());
        }).until(this::isAtTarget).withTimeout(timeoutMs / 1000.0);
    }

    public void resetTurret() {
        setTarget(0.0);
    }

    public Command resetTurretCommand() {
        return run(() -> {
            resetTurret();
        }).until(this::isAtTarget);
    }

    private final TrajectoryCalculations trajCalc = new TrajectoryCalculations();

    public void aimAtHub() {
        setTarget(trajCalc.getNeededYaw());
    }

    public Command aimAtHubCommand() {
        return run(this::aimAtHub);
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
        double relativeTurretAngle = globalAngleToTarget - robotHeading;

        setTarget(relativeTurretAngle);
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
    }


    @Override
    public void periodic() {
        double currentAngle = getTurretAngle();
        double pidOutput = TurretSubsystem.pidController.calculate(currentAngle, targetAngle);
        double feedforwardOutput = TurretSubsystem.feedforward.calculate(
                TurretSubsystem.pidController.getSetpoint().position,
                TurretSubsystem.pidController.getSetpoint().velocity);

        driveMotor.setVoltage(pidOutput + feedforwardOutput);

        recordOutput("Turret Angle", currentAngle);
        recordOutput("Target Angle", targetAngle);
    }

    @Override
    public Sendable log() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
}
