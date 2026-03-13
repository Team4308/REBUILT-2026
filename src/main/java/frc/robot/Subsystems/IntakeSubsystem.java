package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX roller = new TalonFX(Ports.Intake.kRollerMotorId);
  private final TalonFX pivot = new TalonFX(Ports.Intake.kPivotMotorId);

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

  public final static ArmFeedforward feedforward = new ArmFeedforward(Constants.Intake.PIVOT_KS,
      Constants.Intake.PIVOT_KG, Constants.Intake.PIVOT_KV, Constants.Intake.PIVOT_KA);
  public final static ProfiledPIDController pidController = new ProfiledPIDController(
      Constants.Intake.PIVOT_KP, Constants.Intake.PIVOT_KI, Constants.Intake.PIVOT_KD,
      new TrapezoidProfile.Constraints(Constants.Intake.MAX_VEL_DEG_PER_SEC, Constants.Intake.MAX_ACCEL_DEG_PER_SEC2));

  public IntakeSubsystem() {
    configureRoller();
  }

  /* ---------------- Roller ---------------- */

  public void setRollerSpeedA(Supplier<Double> rpm) {
    Logger.recordOutput("Subsystems/Intake/Target Roller Speed", rpm.get());
    roller.setControl(
        rollerRequest.withVelocity(rpm.get() * 60.0));
  }

  public void stopRoller() {
    roller.stopMotor();
  }

  /* ---------------- Pivot ---------------- */

  public void setIntakeAngle(double angleDeg) {
    targetAngleDeg = MathUtil.clamp(angleDeg, 0, 127);
  }

  public double getIntakeAngle() {
    return rotToDeg(pivot.getPosition().getValueAsDouble());
  }

  public boolean isAtAngle() {
    double currentDeg = rotToDeg(pivot.getPosition().getValueAsDouble());
    return Math.abs(currentDeg - targetAngleDeg) < Constants.Intake.ANGLE_TOLERANCE_DEG
        && pivot.getVelocity().getValueAsDouble() < Constants.Intake.VELOCITY_TOLERANCE;
  }

  public void resetIntake() {
    if (pivot.getSupplyCurrent().getValueAsDouble() < 3) {
      pivot.setVoltage(-2.0);
    } else {
      pivot.setVoltage(0);
    }
  }

  public Command resetIntakeCommand() {
    return run(this::resetIntake)
        .until(() -> pivot.getSupplyCurrent().getValueAsDouble() > 3)
        .andThen(new InstantCommand(() -> setIntakeAngle(0)))
        .andThen(new InstantCommand(() -> pivot.setPosition(0)));
  }

  public void stopMotors() {
    targetAngleDeg = rotToDeg(pivot.getPosition().getValueAsDouble());
    setRollerSpeedA(() -> 0.);
    roller.stopMotor();
    pivot.stopMotor();
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

    roller.getConfigurator().apply(cfg);
  }

  private double rotToDeg(double rot) {
    return rot / Constants.Intake.PIVOT_GEAR_RATIO * 360.0 + offset;
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

    double currentDeg = rotToDeg(pivot.getPosition().getValueAsDouble());
    double pidOutput = pidController.calculate(currentDeg, targetAngleDeg);
    double ffOutput = feedforward.calculate(pidController.getSetpoint().position,
        pidController.getSetpoint().velocity);
    double voltage = pidOutput + ffOutput;
    pivot.setVoltage(voltage);
    Logger.recordOutput("Subsystems/Intake/CurrentIntakeAngle", currentDeg);
    Logger.recordOutput("Subsystems/Intake/TargetIntakeAngle", pidController.getSetpoint().position);
    Logger.recordOutput("Subsystems/Intake/RollerSpeed", roller.getVelocity().getValueAsDouble());
  }
}
