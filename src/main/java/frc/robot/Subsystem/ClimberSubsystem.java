package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;

import com.ctre.phoenix6.controls.Follower;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    
    public enum States{
        NOT_CLIMBING,
        GOING_TO_CLIMB,
        IN_CLIMB
    }


    private States currentState = States.NOT_CLIMBING;
    private boolean usingState;
    public TalonFx motor1 = new Talonfx(Constants.ClimberConstants.ClimberMotor1);
    public TalonFx motor2 = new Talonfx(Constants.ClimberConstants.ClimberMotor2);
    private double startRevoulition;
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public ClimberSubsystem(){
        startAngle = GetAngle();
        motor2.setControl(new Follower(motor1.getDeviceID(), false));
    }

    public void PrepareToClimb(){
        MoveToRevolution(Constants.ClimberConstants.RotationsToHighTarget);
    }

    public void Climb(){
        MoveToRevolution(Constants.ClimberConstants.RotationsToTarget);
    }
    
    public void retractClimb() {
        MoveToRevolution(startRevoulition)
    }
    
    public boolean isAtTarget(){
        return (Math.abs(GetAngle() - (Constants.ClimberConstants.RotationsToTarget)) >= Constants.ClimberConstants.tolerance);
    }
    private double GetRevoultion(){
        return motor1.getPosition().getValueAsDouble();
    }
    public void setState(String state) {
        if (state == "NOT_CLIMBING"){
            currentState = States.NOT_CLIMBING;
        } else if(state == "GOING_TO_CLIMB") {
            currentState = States.GOING_TO_CLIMB;
        }else if (state == "IN_CLIMB"){
            currentState = States.IN_CLIMB;
        }
    } 
    public void setStateBased(boolean using) {
        
            usingState = using;
        }
        
    private void MoveToRevolution(double angle){
           motor1.setControl(positionRequest.withPosition(angle*(1/9)));
    }
    
    @Override
    public void periodic() {
        if(!usingState) break;
        switch(currentState){
            case NOT_CLIMBING:
                if(GetAngle() == startRevoulition) break;
                MoveToAngle(startAngle);
                break;
            case GOING_TO_CLIMB:
                MoveToAngle(Constants.ClimberConstants.RotationsToHighTarget);
                break;
            case IN_CLIMB:
                MoveToAngle(Constants.ClimberConstants.RotationsToTarget);    
                break;
        }
    }
}
