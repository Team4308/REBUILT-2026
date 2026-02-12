package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Rollers extends SubsystemBase{
    TalonFX motor = new TalonFX(Constants.IndexerMotorId);
    DutyCycleOut output = new DutyCycleOut(0);
    public double RollerSpeed;
    private boolean CanRoll;
    public enum State {
        IDLE,
        AUTO_AIM,
        SHOOT
    }
    private State currentState = State.IDLE;
    public Rollers(){
         // in init function, set slot 0 gains
         var slot0Configs = new Slot0Configs();
         slot0Configs.kS = Constants.IndexerMotorConfigsKs; // Add 0.1 V output to overcome static friction
         slot0Configs.kV = Constants.IndexerMotorConfigsKv; // A velocity target of 1 rps results in 0.12 V output
         slot0Configs.kP = Constants.IndexerMotorConfigsKp; // An error of 1 rps results in 0.11 V output
         slot0Configs.kI = Constants.IndexerMotorConfigsKi; // no output for integrated error
         slot0Configs.kD = Constants.IndexerMotorConfigsKd; // no output for error derivative


         motor.getConfigurator().apply(slot0Configs);
        RollerSpeed = Constants.IntakeSpeed;
    }
    public void setRollerVelocity(double rpm){
       RollerSpeed = rpm;
    }

    public Command feedBalls(){

    return run(()-> RunMotor());
}


    public void RunMotor(){
    
     
      // create a velocity closed-loop re quest, voltage output, slot 0 configs
      final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

      // set velocity to 8 rps, add 0.5 V to overcome gravity
      motor.setControl(m_request.withVelocity(RollerSpeed/60).withFeedForward(0));
    }
       
  public void NotRunMotor(){
  motor.stopMotor();  
}
    

    @Override
    public void periodic() {
        
      switch (currentState) {
           
          case AUTO_AIM:

          RunMotor();
                
                

            case SHOOT:
                RunMotor();

            case IDLE:
                NotRunMotor();
        }
    }
}



