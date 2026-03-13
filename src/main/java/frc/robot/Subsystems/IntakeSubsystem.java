package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.Util.SubsystemVerbosity;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX m_rollerMotor = new TalonFX(Ports.Intake.kRollerMotorId);
  private final TalonFX m_pivotMotor = new TalonFX(Ports.Intake.kPivotMotorId);

  private final VelocityVoltage rollerRequest = new VelocityVoltage(0);

  private double targetAngleDeg = Constants.Intake.RETRACTED_ANGLE_DEG;

  private enum state {
    REST,
    SHOOTING,
    INTAKING
  }

  private state currentState = state.REST;
  private boolean stateBased = false;

  private double offset = 127;

  private final SubsystemVerbosity verbosity;

  public final static ArmFeedforward feedforward = Constants.Intake.feedforward;
  public final static ProfiledPIDController pidController = Constants.Intake.pidController;

  private Supplier<Double> simSupplier;

  private double voltage;

  public IntakeSubsystem() {
    verbosity = SubsystemVerbosity.HIGH;
    configureRoller();
  }

  /* ---------------- Roller ---------------- */

  public void setRollerSpeedA(Supplier<Double> rpm) {
    Logger.recordOutput("Subsystems/Intake/Target Roller Speed", rpm.get());
    m_rollerMotor.setControl(
        rollerRequest.withVelocity(rpm.get() * 60.0));
  }

  public void stopRoller() {
    m_rollerMotor.stopMotor();
  }

  /* ---------------- Pivot ---------------- */

  public void setIntakeAngle(double angleDeg) {
    targetAngleDeg = MathUtil.clamp(angleDeg, 0, 127);
  }

  public double getIntakeAngle() {
    if (Robot.isSimulation()) {
      if (simSupplier == null) {
        return 0;
      }
      return simSupplier.get();
    }
    return rotToDeg(m_pivotMotor.getPosition().getValueAsDouble());
  }

  public boolean isAtAngle() {
    double currentDeg = getIntakeAngle();
    return Math.abs(currentDeg - targetAngleDeg) < Constants.Intake.ANGLE_TOLERANCE_DEG
        && m_pivotMotor.getVelocity().getValueAsDouble() < Constants.Intake.VELOCITY_TOLERANCE;
  }

  public void resetIntake() {
    if (m_pivotMotor.getSupplyCurrent().getValueAsDouble() < 3) {
      m_pivotMotor.setVoltage(-2.0);
    } else {
      m_pivotMotor.setVoltage(0);
    }
  }

  public Command resetIntakeCommand() {
    return run(this::resetIntake)
        .until(() -> m_pivotMotor.getSupplyCurrent().getValueAsDouble() > 3)
        .andThen(new InstantCommand(() -> setIntakeAngle(0)))
        .andThen(new InstantCommand(() -> m_pivotMotor.setPosition(0)));
  }

  public void stopMotors() {
    targetAngleDeg = rotToDeg(m_pivotMotor.getPosition().getValueAsDouble());
    setRollerSpeedA(() -> 0.);
    m_rollerMotor.stopMotor();
    m_pivotMotor.stopMotor();
  }

  /* --------------- Commands ---------------- */

  public Command setRollerSpeed(Supplier<Double> rpmSupplier) {
    return run(() -> setRollerSpeedA(rpmSupplier));
  }

  public Command moveIntakeToAngle(double targetAngle) {
    return run(() -> setIntakeAngle(targetAngle)).until(() -> isAtAngle());
  }

  public Command intake() {
    return moveIntakeToAngle(Constants.Intake.INTAKE_ANGLE_DEG)
        .alongWith(setRollerSpeed(() -> Constants.Intake.ROLLER_INTAKE_RPM));
  }

  public Command retract() {
    return (moveIntakeToAngle(Constants.Intake.RETRACTED_ANGLE_DEG).alongWith(run(() -> stopRoller())))
        .until(() -> isAtAngle());
  }

  public Command retract(double timeoutMs) {
    return retract().withTimeout(timeoutMs / 1000);
  }

  public Command agitate() {
    return (moveIntakeToAngle(Constants.Intake.AGITATE_HIGH_DEG).until(() -> isAtAngle()).andThen(
        moveIntakeToAngle(Constants.Intake.AGITATE_LOW_DEG).until(() -> isAtAngle()))).repeatedly();
  }

  /* ---------------- States ----------------- */

  public void setState(state s) {
    currentState = s;
  }

  public void setStateBased(boolean using) {
    stateBased = using;
  }

  /* ---------------- Helpers ---------------- */

  private void configureRoller() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = Constants.Intake.ROLLER_KP;
    cfg.Slot0.kI = Constants.Intake.ROLLER_KI;
    cfg.Slot0.kD = Constants.Intake.ROLLER_KD;
    cfg.Slot0.kV = Constants.Intake.ROLLER_KV;

    m_rollerMotor.getConfigurator().apply(cfg);
  }

  private double rotToDeg(double rot) {
    return rot / Constants.Intake.PIVOT_GEAR_RATIO * 360.0 + offset;
  }

  private Pose3d getHopperPose() {
    return new Pose3d(
        DoubleUtils.mapRange(getIntakeAngle(), 0, 127, 0, 0.3048),
        0, 0, new Rotation3d());
  }

  private Pose3d getPivotPose() {
    return new Pose3d(-0.292810438, 0, 0.219075, new Rotation3d(0, Math.toRadians(getIntakeAngle()), 0));
  }

  public double getVoltage() {
    return voltage;
  }

  /**
   * This is for Sim only
   * 
   * @return
   */
  public void setSimSupplier(Supplier<Double> supplier) {
    simSupplier = supplier;
  }

  /* ---------------- Periodic --------------- */

  @Override
  public void periodic() {
    if (stateBased) {
      switch (currentState) {
        case REST:
          retract();
        case SHOOTING:
          agitate();
        case INTAKING:
          intake();
      }
    }

    double currentDeg = getIntakeAngle();
    double pidOutput = pidController.calculate(currentDeg, targetAngleDeg);
    double ffOutput = feedforward.calculate(pidController.getSetpoint().position,
        pidController.getSetpoint().velocity);
    voltage = pidOutput + ffOutput;
    m_pivotMotor.setVoltage(voltage);

    if (verbosity == SubsystemVerbosity.LOW || verbosity == SubsystemVerbosity.HIGH) {
      Logger.recordOutput("Subsystems/Intake/Pivot/Is At Angle?", isAtAngle());
      Logger.recordOutput("Subsystems/Intake/Pivot/Current Angle", currentDeg);
      Logger.recordOutput("Subsystems/Intake/Roller/Roller Speed", m_rollerMotor.getVelocity().getValueAsDouble());
      Logger.recordOutput("Subsystems/Intake/Hopper Pose", getHopperPose());
      Logger.recordOutput("Subsystems/Intake/Pivot Pose", getPivotPose());
    }

    if (verbosity == SubsystemVerbosity.HIGH) {
      Logger.recordOutput("Subsystems/Intake/Pivot/PID Volts", pidOutput);
      Logger.recordOutput("Subsystems/Intake/Pivot/FF Volts", ffOutput);
      Logger.recordOutput("Subsystems/Intake/Pivot/Applied Voltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput("Subsystems/Intake/Pivot/Motor Temperature", m_pivotMotor.getDeviceTemp().getValueAsDouble());
      Logger.recordOutput("Subsystems/Intake/Pivot/Current", m_pivotMotor.getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput("Subsystems/Intake/Pivot/Error", pidController.getPositionError());
      Logger.recordOutput("Subsystems/Intake/Pivot/Velocity", m_pivotMotor.getVelocity().getValueAsDouble());
      Logger.recordOutput("Subsystems/Intake/Pivot/Setpoint Angle", pidController.getSetpoint().position);
      Logger.recordOutput("Subsystems/Intake/Pivot/Setpoint Velocity", pidController.getSetpoint().velocity);

      Logger.recordOutput("Subsystems/Intake/Roller/Applied Voltage",
          m_rollerMotor.getMotorVoltage().getValueAsDouble());
      Logger.recordOutput("Subsystems/Intake/Roller/Motor Temperature",
          m_rollerMotor.getDeviceTemp().getValueAsDouble());
      Logger.recordOutput("Subsystems/Intake/Roller/Current", m_rollerMotor.getSupplyCurrent().getValueAsDouble());
      Logger.recordOutput("Subsystems/Intake/Roller/Target Roller Speed",
          m_rollerMotor.getClosedLoopReference().getValueAsDouble());
    }
  }
}