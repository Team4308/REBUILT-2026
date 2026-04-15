package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Current;
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

  private double offset = -Constants.Intake.RETRACTED_ANGLE_DEG;

  private StatusSignal<Current> pivotSupplyCurrent = m_pivotMotor.getSupplyCurrent();
  private StatusSignal<Current> rollerSupplyCurrent = m_rollerMotor.getSupplyCurrent();

  private final SubsystemVerbosity verbosity;

  public final static ArmFeedforward feedforward = Constants.Intake.feedforward;
  public final static ProfiledPIDController pidController = Constants.Intake.pidController;

  // Tuning for Rollers
  public final LoggedNetworkNumber Roller_kP;
  public final LoggedNetworkNumber Roller_kI;
  public final LoggedNetworkNumber Roller_kD;
  public final LoggedNetworkNumber Roller_kV;
  public final LoggedNetworkNumber Roller_kS;

  public final LoggedNetworkNumber Pivot_kP;
  public final LoggedNetworkNumber Pivot_kI;
  public final LoggedNetworkNumber Pivot_kD;
  public final LoggedNetworkNumber Pivot_kV;
  public final LoggedNetworkNumber Pivot_kS;

  public TalonFXConfiguration oldConfig = new TalonFXConfiguration();
  public TalonFXConfiguration tunableConfig = new TalonFXConfiguration();

  // PID and FF

  private Supplier<Double> simSupplier;

  private double voltage;

  private boolean enabled;

  public enum HopperStates {
    EMPTY,
    HALF,
    FULL
  }

  private HopperStates hopperState = HopperStates.EMPTY;

  public IntakeSubsystem(boolean enabled) {
    verbosity = SubsystemVerbosity.HIGH;
    m_pivotMotor.setPosition(0);
    configureRoller();
    pidController.reset(targetAngleDeg);

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 80;
    limitConfigs.StatorCurrentLimitEnable = true;
    limitConfigs.SupplyCurrentLimit = 40;
    limitConfigs.SupplyCurrentLimitEnable = true;
    m_rollerMotor.getConfigurator().apply(limitConfigs);
    m_pivotMotor.getConfigurator().apply(limitConfigs);

    this.enabled = enabled;

    Roller_kP = new LoggedNetworkNumber("/Tuning/Roller_kP", Constants.Intake.ROLLER_KP);
    Roller_kI = new LoggedNetworkNumber("/Tuning/Roller_kI", Constants.Intake.ROLLER_KI);
    Roller_kD = new LoggedNetworkNumber("/Tuning/Roller_kD", Constants.Intake.ROLLER_KD);
    Roller_kV = new LoggedNetworkNumber("/Tuning/Roller_kV", Constants.Intake.ROLLER_KV);
    Roller_kS = new LoggedNetworkNumber("/Tuning/Roller_kS", Constants.Intake.ROLLER_KS);
    Pivot_kP = new LoggedNetworkNumber("/Tuning/Pivot_kP", Constants.Intake.pidController.getP());
    Pivot_kI = new LoggedNetworkNumber("/Tuning/Pivot_kI", Constants.Intake.pidController.getI());
    Pivot_kD = new LoggedNetworkNumber("/Tuning/Pivot_kD", Constants.Intake.pidController.getD());
    Pivot_kV = new LoggedNetworkNumber("/Tuning/Pivot_kV", Constants.Intake.feedforward.getKv());
    Pivot_kS = new LoggedNetworkNumber("/Tuning/Pivot_kS", Constants.Intake.feedforward.getKs());
  }

  /* ---------------- Roller ---------------- */

  public void setRollerSpeed(Supplier<Double> rpm) {
    Logger.recordOutput("Subsystems/Intake/Target Roller Speed",
        (rpm.get() / -60.0) * Constants.Intake.ROLLER_GEAR_RATIO);
    if (!enabled)
      return;
    m_rollerMotor.setControl(
        rollerRequest.withVelocity((rpm.get() / -60.0) * Constants.Intake.ROLLER_GEAR_RATIO));

  }

  public void setHopperState(HopperStates state) {
    hopperState = state;
  }

  public HopperStates getHopperState() {
    return hopperState;
  }

  public void stopRoller() {
    m_rollerMotor.stopMotor();
  }

  public void setPivotVoltage(double voltage) {
    m_pivotMotor.setVoltage(voltage);
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  public double getPivotSupplyCurrent() {
    return pivotSupplyCurrent.getValueAsDouble();
  }

  public double getRollerSupplyCurrent() {
    return rollerSupplyCurrent.getValueAsDouble();
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
    return rotToDeg(m_pivotMotor.getPosition().getValueAsDouble()) - offset;
  }

  public boolean isAtAngle() {
    double currentDeg = getIntakeAngle();
    return Math.abs(currentDeg - targetAngleDeg) < Constants.Intake.ANGLE_TOLERANCE_DEG;
  }

  public boolean isAtAngleStopped() {
    double currentDeg = getIntakeAngle();
    return Math.abs(currentDeg - targetAngleDeg) < Constants.Intake.ANGLE_TOLERANCE_DEG
        && m_pivotMotor.getVelocity().getValueAsDouble() < Constants.Intake.VELOCITY_TOLERANCE;
  }

  public void stopMotors() {
    targetAngleDeg = rotToDeg(m_pivotMotor.getPosition().getValueAsDouble());
    setRollerSpeed(() -> 0.);
    m_rollerMotor.stopMotor();
    m_pivotMotor.stopMotor();
  }

  /* ---------------- Helpers ---------------- */

  private void configureRoller() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = Constants.Intake.ROLLER_KP;
    cfg.Slot0.kI = Constants.Intake.ROLLER_KI;
    cfg.Slot0.kD = Constants.Intake.ROLLER_KD;
    cfg.Slot0.kS = Constants.Intake.ROLLER_KS;
    cfg.Slot0.kV = Constants.Intake.ROLLER_KV;
    oldConfig = cfg;
    m_rollerMotor.getConfigurator().apply(cfg);
  }

  private void updateRollerConfig() {
    tunableConfig.Slot0.kP = Roller_kP.getAsDouble();
    tunableConfig.Slot0.kI = Roller_kI.getAsDouble();
    tunableConfig.Slot0.kD = Roller_kD.getAsDouble();
    tunableConfig.Slot0.kS = Roller_kS.getAsDouble();
    tunableConfig.Slot0.kV = Roller_kV.getAsDouble();

    m_rollerMotor.getConfigurator().apply(tunableConfig);
    oldConfig = tunableConfig;
  }

  private void updatePivotConfig() {
    pidController.setP(Pivot_kP.getAsDouble());
    pidController.setI(Pivot_kI.getAsDouble());
    pidController.setD(Pivot_kD.getAsDouble());
    feedforward.setKv(Pivot_kV.getAsDouble());
    feedforward.setKs(Pivot_kS.getAsDouble());

  }

  private double rotToDeg(double rot) {
    return rot / Constants.Intake.PIVOT_GEAR_RATIO * 360.0;
  }

  private Pose3d getHopperPose() {
    return new Pose3d(
        DoubleUtils.mapRange(getIntakeAngle(), 0, 127, 0, -0.3048),
        0, 0, new Rotation3d());
  }

  private Pose3d getPivotPose() {
    return new Pose3d(0.292810438, 0, 0.219075, new Rotation3d(0, Math.toRadians(-getIntakeAngle()), 0));
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
    // Tuning
    if (Constants.Intake.TUNING_MODE && !oldConfig.equals(tunableConfig)) {
      updateRollerConfig();
    }

    if (Constants.Intake.TUNING_MODE &&
        (pidController.getP() != Pivot_kP.getAsDouble() ||
            pidController.getI() != Pivot_kI.getAsDouble() ||
            pidController.getD() != Pivot_kD.getAsDouble() ||
            feedforward.getKv() != Pivot_kV.getAsDouble() ||
            feedforward.getKs() != Pivot_kS.getAsDouble())) {
      updatePivotConfig();
    }

    targetAngleDeg = MathUtil.clamp(targetAngleDeg, 0, 127);
    double currentDeg = getIntakeAngle();

    double pidOutput = pidController.calculate(currentDeg, targetAngleDeg);
    double ffOutput = feedforward.calculate(pidController.getSetpoint().position,
        pidController.getSetpoint().velocity);
    voltage = pidOutput + ffOutput;
    // if (enabled)
    // m_pivotMotor.setVoltage(voltage);

    if (verbosity == SubsystemVerbosity.LOW || verbosity == SubsystemVerbosity.HIGH) {
      Logger.recordOutput("Subsystems/Intake/Pivot/Is At Angle?", isAtAngle());
      Logger.recordOutput("Subsystems/Intake/Pivot/Current Angle", currentDeg);
      Logger.recordOutput("Subsystems/Intake/Roller/Roller Speed", m_rollerMotor.getVelocity().getValueAsDouble());
      Logger.recordOutput("Subsystems/Intake/Hopper Pose", getHopperPose());
      Logger.recordOutput("Subsystems/Intake/Pivot Pose", getPivotPose());
      Logger.recordOutput("Subsystems/Intake/Hopper State", hopperState.toString());
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
      Logger.recordOutput("Subsystems/Intake/Pivot/Target Angle", targetAngleDeg);

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
