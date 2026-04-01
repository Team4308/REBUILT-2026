package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.Util.SubsystemVerbosity;

import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX m_ballTunnelMotor = new TalonFX(Ports.Indexer.kBallTunnelMotorId);
    private final TalonFX m_hopperMotor1 = new TalonFX(Ports.Indexer.kHopperMotor2Id);
    private final TalonFX m_hopperMotor2 = new TalonFX(Ports.Indexer.kHopperMotor1Id);

    private final DigitalInput m_beambreak = new DigitalInput(Ports.Indexer.kBeamBreakId);

    private double targetHopperVelocity = 0;
    private double targetBallTunnelVelocity = 0;

    private final VelocityVoltage m_hopperRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_indexerRequest = new VelocityVoltage(0).withSlot(0);

    private SubsystemVerbosity verbosity;

    private boolean enabled;

    private double timeout = 0;

    public IndexerSubsystem(boolean enabled) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = Constants.Indexer.HOPPER_Ks;
        slot0Configs.kV = Constants.Indexer.HOPPER_Kv;
        slot0Configs.kP = Constants.Indexer.HOPPER_Kp;
        slot0Configs.kI = Constants.Indexer.HOPPER_Ki;
        slot0Configs.kD = Constants.Indexer.HOPPER_Kd;

        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;
        limitConfigs.SupplyCurrentLimit = 60;
        limitConfigs.SupplyCurrentLimitEnable = true;
        m_hopperMotor1.getConfigurator().apply(limitConfigs);
        m_hopperMotor2.getConfigurator().apply(limitConfigs);
        m_ballTunnelMotor.getConfigurator().apply(limitConfigs);

        m_hopperMotor1.getConfigurator().apply(slot0Configs);
        m_hopperMotor2.getConfigurator().apply(slot0Configs);

        var motorConfigs = new MotorOutputConfigs();
        motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        m_hopperMotor1.getConfigurator().apply(motorConfigs);

        // m_hopperMotor2.setControl(new Follower(m_hopperMotor1.getDeviceID(),
        // MotorAlignmentValue.Opposed));

        var slot1Configs = new Slot0Configs();
        slot1Configs.kS = Constants.Indexer.BALL_TUNNEL_Ks;
        slot1Configs.kV = Constants.Indexer.BALL_TUNNEL_Kv;
        slot1Configs.kP = Constants.Indexer.BALL_TUNNEL_Kp;
        slot1Configs.kI = Constants.Indexer.BALL_TUNNEL_Ki;
        slot1Configs.kD = Constants.Indexer.BALL_TUNNEL_Kd;
        m_ballTunnelMotor.getConfigurator().apply(slot1Configs);

        var motorConfigs2 = new MotorOutputConfigs();
        motorConfigs2.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs2.NeutralMode = NeutralModeValue.Coast;
        m_ballTunnelMotor.getConfigurator().apply(motorConfigs2);
        m_hopperMotor2.getConfigurator().apply(motorConfigs2);

        verbosity = SubsystemVerbosity.LOW;

        this.enabled = enabled;
    }

    public void setIndexerVelocity(double rpmHopper, double rpmTunnel) {
        targetHopperVelocity = rpmHopper;
        targetBallTunnelVelocity = rpmTunnel;
        if (!enabled) {
            return;
        }
        double motorRPS = (rpmHopper * Constants.Indexer.HOPPER_GEAR_RATIO) / 60.0;
        double motorRPS2 = (rpmTunnel * Constants.Indexer.BALL_TUNNEL_GEAR_RATIO) / 60.0;
        if (m_hopperMotor1.getTorqueCurrent().getValueAsDouble() > Constants.Indexer.EJECT_CURRENT) {
            timeout = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - timeout > Constants.Indexer.EJECT_SECONDS || Robot.isSimulation()) {
            m_hopperMotor1.setControl(m_hopperRequest.withVelocity(motorRPS));
            m_hopperMotor2.setControl(m_hopperRequest.withVelocity(motorRPS));
        } else {
            m_hopperMotor1.setControl(m_hopperRequest.withVelocity(Constants.Indexer.EJECT_SPEED));
            m_hopperMotor2.setControl(m_hopperRequest.withVelocity(Constants.Indexer.EJECT_SPEED));
        }
        m_ballTunnelMotor.setControl(m_indexerRequest.withVelocity(motorRPS2));
    }

    public void stopMotors() {
        targetHopperVelocity = 0;
        targetBallTunnelVelocity = 0;
        m_ballTunnelMotor.stopMotor();
        m_hopperMotor1.stopMotor();
        m_hopperMotor2.stopMotor();
    }

    public double getTargetBallTunnelVelocity() {
        return targetBallTunnelVelocity;
    }

    public double getTargetHopperVelocity() {
        return targetHopperVelocity;
    }

    public double getBallTunnelVelocity() {
        return m_ballTunnelMotor.getVelocity().getValueAsDouble()
                / Constants.Indexer.BALL_TUNNEL_GEAR_RATIO * 60.0;
    }

    public boolean getBeambreak() {
        return m_beambreak.get();
    }

    @Override
    public void periodic() {
        if (verbosity == SubsystemVerbosity.LOW || verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Velocity",
                    m_ballTunnelMotor.getVelocity().getValueAsDouble()
                            / Constants.Indexer.BALL_TUNNEL_GEAR_RATIO * 60.0);
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Target Velocity", targetBallTunnelVelocity);

            Logger.recordOutput("Subsystems/Indexer/Hopper/Velocity",
                    m_hopperMotor1.getVelocity().getValueAsDouble() / Constants.Indexer.HOPPER_GEAR_RATIO * 60.0);
            Logger.recordOutput("Subsystems/Indexer/Hopper/Target Velocity", targetHopperVelocity);
        }

        if (verbosity == SubsystemVerbosity.HIGH) {
            double ballTunnelVelocity = m_ballTunnelMotor.getVelocity().getValueAsDouble()
                    / Constants.Indexer.BALL_TUNNEL_GEAR_RATIO * 60.0;
            double ballTunnelError = targetBallTunnelVelocity - ballTunnelVelocity;
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Error", ballTunnelError);
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Voltage",
                    m_ballTunnelMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Current",
                    m_ballTunnelMotor.getTorqueCurrent().getValueAsDouble());

            double hopperVelocity = m_hopperMotor1.getVelocity().getValueAsDouble()
                    / Constants.Indexer.HOPPER_GEAR_RATIO * 60.0;
            double hopperError = targetHopperVelocity - hopperVelocity;
            Logger.recordOutput("Subsystems/Indexer/Hopper/Error", hopperError);
            Logger.recordOutput("Subsystems/Indexer/Hopper/Voltage",
                    m_hopperMotor1.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Indexer/Hopper/Current",
                    m_hopperMotor1.getTorqueCurrent().getValueAsDouble());
        }
    }
}
