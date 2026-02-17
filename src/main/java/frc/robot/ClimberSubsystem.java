package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.controls.Follower;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    public TalonFx motor1 = new Talonfx(1);
    public TalonFx motor2 = new Talonfx(2);
    public double RotationsToMax = 20;
    public double RotationsToTarget = 15;
    public double startAngle;
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public ClimberSubsystem(){
        startAngle = GetAngle();
        motor2.setControl(new Follower(motor1.getDeviceID(), false));
    }
    
    public void extendClimb() {
        motor1.setControl(positionRequest.withPosition(RotationsToMax));
    }

    public void retractClimb() {
        motor1.setControl(positionRequest.withPosition(startAngle/360));
    }

    public void Climb() {
        motor1.setControl(positionRequest.withPosition(RotationsToTarget));
    }
    public boolean isAtTarget(){
        return (Math.abs(GetAngle() - (RotationsToTarget*360)) >= Constants.tolerance);

    }
    private double GetAngle(){
        return motor1.getPosition().getValueAsDouble() * (1/25)*360;
    }
}
