package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbersubsystem extends SubsystemBase {

    private final TalonFX motor1 = new TalonFX(1);  // Leader
    private final TalonFX motor2 = new TalonFX(2); // Follower

    public double Revolutions = 15.0;
    public double StartingPos = 0;
    public double Tolerance = 5;
    public double Fullextend = 20;

    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public Climbersubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Optional but recommended for climbers
        config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

        motor1.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config);

        // Make TalonFX 2 follow TalonFX 1    
        motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Aligned));
        StartingPos = currentAngle();
    }

    public void Climb(){
        motor1.setControl(positionRequest.withPosition(Revolutions));
    }
    public void retractClimb(){
        motor1.setControl(positionRequest.withPosition(StartingPos/360));
    }
    public void extendClimb(){
        motor1.setControl(positionRequest.withPosition(Fullextend));
    }
    public Boolean isAtTarget(){
        return (Math.abs(currentAngle()-Revolutions*360/25) <= Constants.tolerance);
    }
    private Double currentAngle(){
        return  motor1.getPosition().getValueAsDouble() * (1/25) * 360;
    }
    
}