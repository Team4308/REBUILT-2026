package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.trajectories.shooter.ShooterSystem;
import ca.team4308.absolutelib.subsystems.Arm;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.TrajectoryCalculations;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;

import java.util.function.Supplier;

public class HoodSubsystem extends SubsystemBase {
    private final TalonFX m_hoodMotor = new TalonFX(Constants.Hood.HoodMotor);

    

    private double targetAngle = 0;
    private StateManager.RobotState currentState = StateManager.RobotState.Idle;
    private boolean isStateManaged = false;
    public HoodSubsystem() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Software Limits
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (Constants.Hood.FORWARD_SOFT_LIMIT_ANGLE / 360.0) * Constants.Hood.TOTAL_GEAR_RATIO;
        talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (Constants.Hood.REVERSE_SOFT_LIMIT_ANGLE / 360.0) * Constants.Hood.TOTAL_GEAR_RATIO;
        talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        m_hoodMotor.getConfigurator().apply(talonFXConfigs);
        m_hoodMotor.setPosition(0);
    }

    public double getHoodAngle() {
        return (m_hoodMotor.getPosition().getValueAsDouble() / Constants.Hood.TOTAL_GEAR_RATIO) * 360.0;
    }

    public void setHoodAngle(double angle) {
        this.targetAngle = Math.min(Math.max(angle, Constants.Hood.REVERSE_SOFT_LIMIT_ANGLE), Constants.Hood.FORWARD_SOFT_LIMIT_ANGLE);
    }

    public boolean isAtPosition() {
        // Uses a tolerance value from Constants 
        return Math.abs(getHoodAngle() - targetAngle) < Constants.Hood.kToleranceDegrees;
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
        if (m_hoodMotor.getSupplyCurrent().getValueAsDouble() < Constants.Hood.ampThreshold) {
            m_hoodMotor.setVoltage(-2.0); 
        } else {
            m_hoodMotor.setVoltage(0);
            m_hoodMotor.setPosition(0);
        }
    }

    public Command resetHoodCommand() {
        return run(this::resetHood).until(() -> m_hoodMotor.getSupplyCurrent().getValueAsDouble() > 20.0).andThen(runOnce(() -> m_hoodMotor.setPosition(0)));
    }

    public void setState(String state) { this.currentState = state; }
    public void setStateBased(boolean using) { this.isStateManaged = using; }

    // Placeholder for Vision/Strategy Logic
    public void aimAtHub() { setHoodAngle( new TrajectoryCalculations().getNeededPitch()); }
    public Command aimAtHubCommand() { return run(this::aimAtHub); }

    public void aimAtPassingZone() { setHoodAngle( Constants.Hood.kPassingAngle ); }
    public Command aimAtPassingZoneCommand() { return run(this::aimAtPassingZone); }

    public class StateManager extends SubsystemBase {
            //Robot states (I think these are all, check pls)
            public enum RobotState {
                Idle,
                Intake,
                Shoot,
                AutoAim,
                Climb
            }

            private RobotState currenState = RobotState.Idle;
            //Subsystem reference to access them
            private final HoodSubsystem hoodSubsystem;
            private final ShooterSystem shooterSystem;
            private final IntakeSystem IntakeSystem;
            private final Arm arm;
            //add all the subsystems here: before the files are merged, I'll add placeholder values for subsystems

            public void StateManager(ShooterSystem shooterSystem, IntakeSystem IntakeSystem, Arm arm) {
                this.shooterSystem = shooterSystem;
                this.IntakeSystem = IntakeSystem;
                this.arm = arm;
            }

            public void setState(StateManager.RobotState state) {
                this.currentState = state;
            }


            public RobotState getState() {
                return currenState;
            }   
            
        }

    @Override
    public void periodic() {

        switch (currentState) {
            case Idle:
                ShooterSystem.StopMotors();
                IntakeSystem.StopMotors();
                Arm.StopMotors();
                break;
            
            case Intake:
                IntakeSystem.setSpeed(1);
                ShooterSystem.StopMotors();
                Arm.setTargetAngle(15);
                break;
            
            case Shoot:
                ShooterSystem.setTargetRPM(4500);
                Arm.setTargetAngle(55);
                IntakeSystem.setSpeed(0.2); //I think it should still move since we need to feed the turret
                break;
            case AutoAim:
                ShooterSystem.setTargetRPM(4500); //I don't know how much rpm we need so like change this according to need
                break;
            case Climb:
                Arm.setTargetPosition(1);
                ShooterSystem.StopMotors();
                IntakeSystem.StopMotors();
                break;
        }

        double currentAngle = getHoodAngle();
        double pidOutput = Constants.Hood.pidController.calculate(currentAngle, targetAngle);
        double ffVolts = Constants.Hood.feedforward.calculate(Units.degreesToRadians(currentAngle), Constants.Hood.pidController.getSetpoint().velocity);

        m_hoodMotor.setVoltage(pidOutput + ffVolts);

        Logger.recordOutput("Subsystems/Hood/TargetAngle", targetAngle);
        Logger.recordOutput("Subsystems/Hood/CurrentAngle", currentAngle);
        Logger.recordOutput("Subsystems/Hood/State", currentState);
    }
    public void StopMotors() {
        m_hoodMotor.setVoltage(0);
        m_hoodMotor.setPosition(0); 
    }
}