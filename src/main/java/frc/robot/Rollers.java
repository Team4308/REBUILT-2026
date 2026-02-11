package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class Rollers {
    TalonFX motor = new TalonFX(1);
    DutyCycleOut output = new DutyCycleOut(0);
    public double RollerSpeed;
    public Rollers(){
         // in init function, set slot 0 gains
         var slot0Configs = new Slot0Configs();
         slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
         slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
         slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
         slot0Configs.kI = 0; // no output for integrated error
         slot0Configs.kD = 0; // no output for error derivative


         motor.getConfigurator().apply(slot0Configs);
        RollerSpeed = Constants.IntakeSpeed;
    }
    public void setRollerVelocity(double rpm){
       RollerSpeed = rpm;
    }

    public void feedBalls(){
      
      // create a velocity closed-loop request, voltage output, slot 0 configs
      final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

      // set velocity to 8 rps, add 0.5 V to overcome gravity
      motor.setControl(m_request.withVelocity(RollerSpeed/60).withFeedForward(0));


}

 
    
}
