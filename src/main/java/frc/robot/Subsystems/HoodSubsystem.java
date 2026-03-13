package frc.robot.Subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.Util.SubsystemVerbosity;
import frc.robot.Util.TrajectoryCalculations;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX m_hoodMotor = new TalonFX(Ports.Shooting.Hood.kHoodId);

    private double targetAngle = 7.5;

    private double angleOffset = Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE;

    public enum RobotState {
        REST,
        PASS_RIGHT,
        PASS_LEFT,
        SHOOT,
    }

    private RobotState currentState = RobotState.REST;

    private boolean usingState = false;

    private TrajectoryCalculations trajectory;

    private SubsystemVerbosity verbosity;

    private Supplier<Double> turretSupplier;
    private Supplier<Double> simSupplier;
    private double voltage;

    private ProfiledPIDController pidController = Constants.Shooting.Hood.pidController;

    public HoodSubsystem() {
        trajectory = null;
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_hoodMotor.getConfigurator().apply(talonFXConfigs);
        m_hoodMotor.setPosition(0);

        verbosity = SubsystemVerbosity.HIGH;

        if (Robot.isSimulation()) { // Brute force sim
            pidController.setP(1);
        }
    }

    public double getVoltage() {
        return voltage;
    }

    public double getHoodAngle() {
        if (Robot.isSimulation()) {
            if (simSupplier == null) {
                return 0;
            }
            return simSupplier.get();
        }
        return angleOffset
                + (m_hoodMotor.getPosition().getValueAsDouble() / Constants.Shooting.Hood.TOTAL_GEAR_RATIO) * 360.0;
    }

    public Command setHoodAngleCommand(double angle) {
        return run(() -> setHoodAngle(angle)).until(this::isAtPosition);
    }

    public void setHoodAngle(double angle) {
        targetAngle = MathUtil.clamp(
                angle,
                Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE,
                Constants.Shooting.Hood.FORWARD_SOFT_LIMIT_ANGLE);
    }

    public boolean isAtPosition() {
        // Uses a tolerance value from Constants
        return Math.abs(getHoodAngle() - targetAngle) < Constants.Shooting.Hood.TOLERANCE_DEGREES
                && m_hoodMotor.getVelocity().getValueAsDouble() < Constants.Shooting.Hood.TOLERANCE_VELOCITY;
    }

    // Move to angle (Supplier allows for dynamic targets like Limelight)
    public Command moveHood(Supplier<Double> angleSupplier) {
        return run(() -> setHoodAngle(angleSupplier.get())).until(this::isAtPosition);
    }

    // Move to angle with Timeout
    public Command moveHood(Supplier<Double> angleSupplier, double timeoutSeconds) {
        return moveHood(angleSupplier).withTimeout(timeoutSeconds);
    }

    public void resetHood() {
        if (m_hoodMotor.getSupplyCurrent().getValueAsDouble() < Constants.Shooting.Hood.AMP_THRESHOLD) {
            m_hoodMotor.setVoltage(-2.0);
        } else {
            m_hoodMotor.setVoltage(0);
        }
    }

    public Command resetHoodCommand() {
        return run(this::resetHood)
                .until(() -> m_hoodMotor.getSupplyCurrent().getValueAsDouble() > Constants.Shooting.Hood.AMP_THRESHOLD)
                .andThen(new InstantCommand(() -> setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE)))
                .andThen(new InstantCommand(() -> m_hoodMotor.setPosition(0)));
    }

    public void setTrajectoryCalculations(TrajectoryCalculations trajectoryCalc) {
        this.trajectory = trajectoryCalc;
    }

    public void setState(RobotState state) {
        this.currentState = state;
    }

    public RobotState getState() {
        return currentState;
    }

    public void setUsingState(boolean using) {
        usingState = using;
    }

    public void stopMotors() {
        m_hoodMotor.setVoltage(0);
        targetAngle = getHoodAngle();
    }

    private Alliance getAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return Alliance.Blue;
        }

        return alliance.get();
    }

    private void restingState() {
        setHoodAngle(Constants.Shooting.Hood.REVERSE_SOFT_LIMIT_ANGLE);
    }

    private void passRightState() {
        if (trajectory == null || !trajectory.suppliersAreSet())
            return;
        final Translation3d passRightPose = (getAlliance() == Alliance.Red)
                ? new Translation3d(
                        FieldLayout.kFieldLength - FieldLayout.ShooterTargets.kPASS_RIGHT_POSE.getX(),
                        FieldLayout.kFieldWidth - FieldLayout.ShooterTargets.kPASS_RIGHT_POSE.getY(),
                        FieldLayout.ShooterTargets.kPASS_RIGHT_POSE.getZ())
                : FieldLayout.ShooterTargets.kPASS_RIGHT_POSE;
        trajectory.setTargetSupplier(() -> {
            return passRightPose;
        });
        trajectory.updateShot();
        setHoodAngle(trajectory.getNeededPitch());
    }

    private void passLeftState() {
        if (trajectory == null || !trajectory.suppliersAreSet())
            return;
        final Translation3d passLeftPose = (getAlliance() == Alliance.Red)
                ? new Translation3d(
                        FieldLayout.kFieldLength - FieldLayout.ShooterTargets.kPASS_LEFT_POSE.getX(),
                        FieldLayout.kFieldWidth - FieldLayout.ShooterTargets.kPASS_LEFT_POSE.getY(),
                        FieldLayout.ShooterTargets.kPASS_LEFT_POSE.getZ())
                : FieldLayout.ShooterTargets.kPASS_LEFT_POSE;
        trajectory.setTargetSupplier(() -> {
            return passLeftPose;
        });
        trajectory.updateShot();
        setHoodAngle(trajectory.getNeededPitch());
    }

    private void shootHubState() {
        if (trajectory == null || !trajectory.suppliersAreSet())
            return;
        final Translation3d hubPose = (getAlliance() == Alliance.Red)
                ? FieldLayout.ShooterTargets.kRED_HUB_POSE
                : FieldLayout.ShooterTargets.kBLUE_HUB_POSE;
        trajectory.setTargetSupplier(() -> {
            return hubPose;
        });
        trajectory.updateShot();
        setHoodAngle(trajectory.getNeededPitch());
    }

    public void setTurretSupplier(Supplier<Double> turretSupplier) {
        this.turretSupplier = turretSupplier;
    }

    /**
     * This is for Sim only
     * 
     * @return
     */
    public void setSimSupplier(Supplier<Double> supplier) {
        simSupplier = supplier;
    }

    private Pose3d getHoodPose() {
        double turretYawRad = turretSupplier.get();
        double offsetX = 0.109474;
        double offsetZ = 0.08255;
        double rotatedX = offsetX * Math.cos(turretYawRad);
        double rotatedY = offsetX * Math.sin(turretYawRad);
        double pitchRad = Math.toRadians(getHoodAngle() - 7.5);
        return new Pose3d(
                0.1362075 + rotatedX, rotatedY, 0.3370134992 + offsetZ,
                new Rotation3d(0, pitchRad, turretYawRad));
    }

    @Override
    public void periodic() {
        boolean underTrench = NetworkTableInstance.getDefault()
                .getTable("AdvantageKit/RealOutputs")
                .getEntry("Swerve/UnderTrench")
                .getBoolean(false);
        // Safety override: hood must retract under trench
        if (underTrench) {
            restingState();
        } else if (usingState) {
            switch (currentState) {
                case REST:
                    restingState();
                    break;

                case SHOOT:
                    shootHubState();
                    break;

                case PASS_RIGHT:
                    passRightState();
                    break;

                case PASS_LEFT:
                    passLeftState();
                    break;

                default:
                    restingState();
                    break;
            }
        }

        double currentAngle = getHoodAngle();
        double pidOutput = pidController.calculate(currentAngle, targetAngle);
        double ffVolts = Constants.Shooting.Hood.feedforward.calculate(
                pidController.getSetpoint().position,
                pidController.getSetpoint().velocity);
        voltage = pidOutput + ffVolts;
        m_hoodMotor.setVoltage(voltage);

        if (verbosity == SubsystemVerbosity.LOW || verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Hood/Is At Target?", isAtPosition());
            Logger.recordOutput("Subsystems/Hood/Angle", currentAngle);

            Logger.recordOutput("Subsystems/Hood/Pose", getHoodPose());
        }

        if (verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Hood/PidVolts", pidOutput);
            Logger.recordOutput("Subsystems/Hood/FFVolts", ffVolts);
            Logger.recordOutput("Subsystems/Hood/Applied Voltage",
                    m_hoodMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Hood/Motor Temperature",
                    m_hoodMotor.getDeviceTemp().getValueAsDouble());
            Logger.recordOutput("Subsystems/Hood/Current", m_hoodMotor.getSupplyCurrent().getValueAsDouble());
            Logger.recordOutput("Subsystems/Hood/Error",
                    Constants.Shooting.Hood.pidController.getPositionError());
            Logger.recordOutput("Subsystems/Hood/Velocity", m_hoodMotor.getVelocity().getValueAsDouble());
            Logger.recordOutput("Subsystems/Hood/Setpoint Angle",
                    Constants.Shooting.Hood.pidController.getSetpoint().position);
            Logger.recordOutput("Subsystems/Hood/Setpoint Velocity",
                    Constants.Shooting.Hood.pidController.getSetpoint().velocity);

        }
    }
}