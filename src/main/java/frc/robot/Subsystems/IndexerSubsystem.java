package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    private final TalonFX m_hopperMotor = new TalonFX(Ports.Indexer.kHopperMotorId);

    private final DigitalInput m_beambreak = new DigitalInput(Ports.Indexer.kBeamBreakId);

    private double targetHopperVelocity = 0;
    private double targetBallTunnelVelocity = 0;

    private final VelocityVoltage m_hopperRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_ballTunnelRequest = new VelocityVoltage(0).withSlot(0);

    private SubsystemVerbosity verbosity;

    private final boolean enabled;

    /** FPGA timestamp of the most recent hopper jam detection. */
    private double lastJamTimestamp = Double.NEGATIVE_INFINITY;

    public IndexerSubsystem(boolean enabled) {
        // Hopper (single motor)
        var hopperConfig = new TalonFXConfiguration();
        hopperConfig.Slot0.kS = Constants.Indexer.HOPPER_Ks;
        hopperConfig.Slot0.kV = Constants.Indexer.HOPPER_Kv;
        hopperConfig.Slot0.kP = Constants.Indexer.HOPPER_Kp;
        hopperConfig.Slot0.kI = Constants.Indexer.HOPPER_Ki;
        hopperConfig.Slot0.kD = Constants.Indexer.HOPPER_Kd;
        applyCurrentLimits(hopperConfig);
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_hopperMotor.getConfigurator().apply(hopperConfig);

        // Ball tunnel
        var ballTunnelConfig = new TalonFXConfiguration();
        ballTunnelConfig.Slot0.kS = Constants.Indexer.BALL_TUNNEL_Ks;
        ballTunnelConfig.Slot0.kV = Constants.Indexer.BALL_TUNNEL_Kv;
        ballTunnelConfig.Slot0.kP = Constants.Indexer.BALL_TUNNEL_Kp;
        ballTunnelConfig.Slot0.kI = Constants.Indexer.BALL_TUNNEL_Ki;
        ballTunnelConfig.Slot0.kD = Constants.Indexer.BALL_TUNNEL_Kd;
        applyCurrentLimits(ballTunnelConfig);
        ballTunnelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        ballTunnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        m_ballTunnelMotor.getConfigurator().apply(ballTunnelConfig);

        verbosity = SubsystemVerbosity.LOW;

        this.enabled = enabled;
    }

    private static void applyCurrentLimits(TalonFXConfiguration config) {
        config.CurrentLimits.StatorCurrentLimit = 160;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    /**
     * Sets hopper and ball tunnel velocities (mechanism RPM). If the hopper is
     * being driven forward and a jam is detected (torque current above
     * {@link Constants.Indexer#EJECT_CURRENT}), the hopper briefly reverses at
     * {@link Constants.Indexer#EJECT_SPEED} for
     * {@link Constants.Indexer#EJECT_SECONDS} to clear it.
     */
    public void setIndexerVelocity(double rpmHopper, double rpmTunnel) {
        targetHopperVelocity = rpmHopper;
        targetBallTunnelVelocity = rpmTunnel;
        if (!enabled) {
            return;
        }

        double now = Timer.getFPGATimestamp();
        boolean feedingForward = rpmHopper > 0;
        if (feedingForward && !Robot.isSimulation()
                && Math.abs(m_hopperMotor.getTorqueCurrent().getValueAsDouble()) > Constants.Indexer.EJECT_CURRENT) {
            lastJamTimestamp = now;
        }

        boolean unjamming = feedingForward && (now - lastJamTimestamp) < Constants.Indexer.EJECT_SECONDS;
        double hopperRpm = unjamming ? Constants.Indexer.EJECT_SPEED : rpmHopper;

        m_hopperMotor.setControl(m_hopperRequest.withVelocity(hopperRpmToMotorRps(hopperRpm)));
        m_ballTunnelMotor.setControl(m_ballTunnelRequest.withVelocity(
                (rpmTunnel * Constants.Indexer.BALL_TUNNEL_GEAR_RATIO) / 60.0));
    }

    public void stopMotors() {
        targetHopperVelocity = 0;
        targetBallTunnelVelocity = 0;
        m_ballTunnelMotor.stopMotor();
        m_hopperMotor.stopMotor();
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

    public double getHopperVelocity() {
        return m_hopperMotor.getVelocity().getValueAsDouble()
                / Constants.Indexer.HOPPER_GEAR_RATIO * 60.0;
    }

    public boolean isUnjamming() {
        return targetHopperVelocity > 0
                && (Timer.getFPGATimestamp() - lastJamTimestamp) < Constants.Indexer.EJECT_SECONDS;
    }

    public boolean getBeambreak() {
        return m_beambreak.get();
    }

    private static double hopperRpmToMotorRps(double rpm) {
        return (rpm * Constants.Indexer.HOPPER_GEAR_RATIO) / 60.0;
    }

    @Override
    public void periodic() {
        if (verbosity == SubsystemVerbosity.LOW || verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Velocity", getBallTunnelVelocity());
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Target Velocity", targetBallTunnelVelocity);

            Logger.recordOutput("Subsystems/Indexer/Hopper/Velocity", getHopperVelocity());
            Logger.recordOutput("Subsystems/Indexer/Hopper/Target Velocity", targetHopperVelocity);
            Logger.recordOutput("Subsystems/Indexer/Hopper/Unjamming", isUnjamming());
        }

        if (verbosity == SubsystemVerbosity.HIGH) {
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Error",
                    targetBallTunnelVelocity - getBallTunnelVelocity());
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Voltage",
                    m_ballTunnelMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Indexer/BallTunnel/Current",
                    m_ballTunnelMotor.getTorqueCurrent().getValueAsDouble());

            Logger.recordOutput("Subsystems/Indexer/Hopper/Error",
                    targetHopperVelocity - getHopperVelocity());
            Logger.recordOutput("Subsystems/Indexer/Hopper/Voltage",
                    m_hopperMotor.getMotorVoltage().getValueAsDouble());
            Logger.recordOutput("Subsystems/Indexer/Hopper/Current",
                    m_hopperMotor.getTorqueCurrent().getValueAsDouble());
        }
    }
}
