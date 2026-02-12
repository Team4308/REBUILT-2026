package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbersubsystem extends SubsystemBase {

    private final TalonFX motor1 = new TalonFX(Constants.Motor1ID);  // Leader
    private final TalonFX motor2 = new TalonFX(Constants.Motor2ID); // Follower

    public double StartingPos = 0;
    
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public Climbersubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Optional but recommended for climbers
        config.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

        motor1.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config);

        // Make TalonFX 2 follow TalonFX 1    
        motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Aligned));
        StartingPos = getCurrentAngle();
    }

    public Command Climb(){
        motor1.setControl(positionRequest.withPosition(Constants.target));
                return null;
    }
    public Command retractClimb(){
        motor1.setControl(positionRequest.withPosition(StartingPos/360));
                return null;
    }
    public Command extendClimb(){
        motor1.setControl(positionRequest.withPosition(Constants.fullExtend));
                return null;
    }
    public Boolean isAtTarget(){
        return (Math.abs(getCurrentAngle()-Constants.target*360/25) <= Constants.tolerance);
    }
    private Double getCurrentAngle(){
        return  motor1.getPosition().getValueAsDouble() * (1/25) * 360;
    }
    public Command StopMotors(){
        motor1.stopMotor();
        motor2.stopMotor();
        return null;
    }
    public Command setState(String state){
        return null;
    }
    public Command setStateBased(boolean using){
        return null;
    }
}