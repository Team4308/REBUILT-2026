package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Util.SubsystemVerbosity;

import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX m_ballTunnelMotor = new TalonFX(Ports.Indexer.kBallTunnelMotorId);
    private final TalonFX m_hopperMotor1 = new TalonFX(Ports.Indexer.kHopperMotor1Id);
    private final TalonFX m_hopperMotor2 = new TalonFX(Ports.Indexer.kHopperMotor2Id);

    private final DigitalInput m_beambreak = new DigitalInput(Ports.Indexer.kBeamBreakId);

    private double targetHopperVelocity = 0;
    private double targetBallTunnelVelocity = 0;

    public enum State { // the diff states
        IDLE,
        SHOOTING
    }

    private State currentState = State.IDLE;

    private boolean usingState = false;

    private final VelocityVoltage m_hopperRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_indexerRequest = new VelocityVoltage(0).withSlot(0);

    private SubsystemVerbosity verbosity;

    public IndexerSubsystem() {
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = Constants.Indexer.HOPPER_Ks; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = Constants.Indexer.HOPPER_Kv; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = Constants.Indexer.HOPPER_Kp; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = Constants.Indexer.HOPPER_Ki; // no output for integrated error
        slot0Configs.kD = Constants.Indexer.HOPPER_Kd; // no output for error derivative
        m_hopperMotor1.getConfigurator().apply(slot0Configs);

        var motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_hopperMotor1.getConfigurator().apply(motorConfigs);

        m_hopperMotor2.setControl(new Follower(m_hopperMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        var slot1Configs = new Slot0Configs();
        slot1Configs.kS = Constants.Indexer.BALL_TUNNEL_Ks; // Add 0.1 V output to overcome static friction
        slot1Configs.kV = Constants.Indexer.BALL_TUNNEL_Kv; // A velocity target of 1 rps results in 0.12 V output
        slot1Configs.kP = Constants.Indexer.BALL_TUNNEL_Kp; // An error of 1 rps results in 0.11 V output
        slot1Configs.kI = Constants.Indexer.BALL_TUNNEL_Ki; // no output for integrated error
        slot1Configs.kD = Constants.Indexer.BALL_TUNNEL_Kd; // no output for error derivative

        m_ballTunnelMotor.getConfigurator().apply(slot1Configs);
        var motorConfigs2 = new MotorOutputConfigs();
        motorConfigs2.Inverted = InvertedValue.CounterClockwise_Positive;
        m_ballTunnelMotor.getConfigurator().apply(motorConfigs2);

        verbosity = SubsystemVerbosity.HIGH;
    }

    public void setHopperVelocity(double rpm) {
        targetHopperVelocity = rpm;
        double motorRPS = (rpm * Constants.Indexer.HOPPER_GEAR_RATIO) / 60.0;
        m_hopperMotor1.setControl(m_hopperRequest.withVelocity(motorRPS));

    }

    public void setIndexerVelocity(double rpm) {
        targetBallTunnelVelocity = rpm;
        double motorRPS = (rpm * Constants.Indexer.BALL_TUNNEL_GEAR_RATIO) / 60.0;
        m_ballTunnelMotor.setControl(m_indexerRequest.withVelocity(motorRPS));

    }

    public void stopMotors() {
        targetHopperVelocity = 0;
        targetBallTunnelVelocity = 0;
        m_ballTunnelMotor.stopMotor();
        m_hopperMotor1.stopMotor();
        m_hopperMotor2.stopMotor();
    }

    public void setState(State newState) {
        this.currentState = newState;
    }

    public void setUsingState(boolean using) {
        usingState = using;
    }

    @Override
    public void periodic() {
        boolean ballsReady = !m_beambreak.get();

        if (usingState) {
            switch (currentState) {
                case IDLE:
                    if (ballsReady) {
                        stopMotors();
                    } else {
                        setHopperVelocity(Constants.Indexer.PASSIVE_HOPEPR_VELOCITY);
                        setIndexerVelocity(Constants.Indexer.PASSIVE_INDEXER_VELOCITY);
                    }
                    break;
                case SHOOTING:
                    setHopperVelocity(Constants.Indexer.DEFAULT_HOPPER_VELOCITY);
                    setIndexerVelocity(Constants.Indexer.DEFAULT_INDEXER_VELOCITY);
                    break;
            }
        }

        if (verbosity == SubsystemVerbosity.LOW || verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Velocity",
                    m_ballTunnelMotor.getVelocity().getValueAsDouble()
                            / Constants.Indexer.BALL_TUNNEL_GEAR_RATIO * 60.0);
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Target Velocity", targetBallTunnelVelocity);

            Logger.recordOutput("Subsystems/Indexer/Hopper/Velocity",
                    m_hopperMotor1.getVelocity().getValueAsDouble() / Constants.Indexer.HOPPER_GEAR_RATIO * 60.0);
            Logger.recordOutput("Subsystems/Hopper/Target Velocity", targetHopperVelocity);
        }

        if (verbosity == SubsystemVerbosity.HIGH) {
            double ballTunnelVelocity = m_ballTunnelMotor.getVelocity().getValueAsDouble()
                    / Constants.Indexer.BALL_TUNNEL_GEAR_RATIO * 60.0;
            double ballTunnelError = targetBallTunnelVelocity - ballTunnelVelocity;
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Error", ballTunnelError);
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Voltage",
                    m_ballTunnelMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Current",
                    m_ballTunnelMotor.getStatorCurrent().getValueAsDouble());

            double hopperVelocity = m_hopperMotor1.getVelocity().getValueAsDouble()
                    / Constants.Indexer.HOPPER_GEAR_RATIO * 60.0;
            double hopperError = targetHopperVelocity - hopperVelocity;
            Logger.recordOutput("Subsystems/Indexer/Hopper/Error", hopperError);
            Logger.recordOutput("Subsystems/Indexer/Hopper/Voltage",
                    m_hopperMotor1.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Indexer/Hopper/Current",
                    m_hopperMotor1.getStatorCurrent().getValueAsDouble());
        }
    }
}
