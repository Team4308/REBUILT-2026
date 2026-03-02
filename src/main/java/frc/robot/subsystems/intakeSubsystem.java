package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX roller = new TalonFX(Constants.Intake.ROLLER_ID);
  private final TalonFX pivot = new TalonFX(Constants.Intake.PIVOT_ID);

  private final VelocityVoltage rollerRequest = new VelocityVoltage(0);
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0);

  private double targetAngleDeg = Constants.Intake.RETRACTED_ANGLE_DEG;

  private enum state {
    REST,
    SHOOTING,
    INTAKING
  }
  private state currentState = state.REST;
  private boolean stateBased = false;

  public IntakeSubsystem() {
    configureRoller();
    configurePivot();

    // Intake always starts retracted
    pivot.setPosition(0);
  }

  /* ---------------- Roller ---------------- */

  public void setRollerSpeed(double rpm) {
    roller.setControl(
        rollerRequest.withVelocity(rpm * 60.0));
  }

  public void stopRoller() {
    roller.stopMotor();
  }

  /* ---------------- Pivot ---------------- */

  public void setIntakeAngle(double angleDeg) {
    targetAngleDeg = angleDeg;
    pivot.setControl(
        pivotRequest.withPosition(degToRot(angleDeg)));
  }

  public boolean isAtAngle() {
    double currentDeg = rotToDeg(pivot.getPosition().getValueAsDouble());
    return Math.abs(currentDeg - targetAngleDeg)
        < Constants.Intake.ANGLE_TOLERANCE_DEG;
  }

  /* --------------- Commands ---------------- */

  public Command setRollerSpeed(Supplier<Double> rpmSupplier) {
    return run(() -> setRollerSpeed(rpmSupplier.get()));
  }

  public Command moveIntakeToAngle(double targetAngle) {
    return run(() -> setIntakeAngle(targetAngle)).until(() -> isAtAngle());
  }

  public Command intake() {
    return moveIntakeToAngle(Constants.Intake.INTAKE_ANGLE_DEG).alongWith(setRollerSpeed(() -> Constants.Intake.ROLLER_INTAKE_RPM));
  }

  public Command retract() {
    return (moveIntakeToAngle(Constants.Intake.RETRACTED_ANGLE_DEG).alongWith(run(() -> stopRoller()))).until(() -> isAtAngle());
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

  private void configurePivot() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = Constants.Intake.PIVOT_KP;
    cfg.Slot0.kI = Constants.Intake.PIVOT_KI;
    cfg.Slot0.kD = Constants.Intake.PIVOT_KD;
    cfg.Slot0.kG = Constants.Intake.PIVOT_KG;
    cfg.Slot0.kV = Constants.Intake.PIVOT_KV;
    cfg.Slot0.kA = Constants.Intake.PIVOT_KA;

    cfg.MotionMagic.MotionMagicCruiseVelocity =
        degToRot(Constants.Intake.MAX_VEL_DEG_PER_SEC);
    cfg.MotionMagic.MotionMagicAcceleration =
        degToRot(Constants.Intake.MAX_ACCEL_DEG_PER_SEC2);

    pivot.getConfigurator().apply(cfg);
  }

  private double degToRot(double deg) {
    return deg / 360.0 * Constants.Intake.PIVOT_GEAR_RATIO;
  }

  private double rotToDeg(double rot) {
    return rot / Constants.Intake.PIVOT_GEAR_RATIO * 360.0;
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
  }
}
