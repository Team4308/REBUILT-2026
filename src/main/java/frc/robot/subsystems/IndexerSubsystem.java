package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IndexerSubsystem extends SubsystemBase{
    
    private final DigitalInput m_beambreak = new DigitalInput(Constants.BeambreakSensor);

    private State currentState = State.IDLE;
    
    private boolean usingState = false;

    //private double slowerIndexerSpeed;
    
    private double hopperSpeed;
    private double indexerSpeed;

    public enum State { //the diff states 
        IDLE,
        SHOOTING
    }
    
    TalonFX Falcon = new TalonFX(Constants.HopperMotorId);
    TalonFX Kraken = new TalonFX(Constants.IndexerMotorId);
    
    private final VelocityVoltage m_hopperRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_indexerRequest = new VelocityVoltage(0).withSlot(0);

    
    public IndexerSubsystem(){
        hopperSpeed = Constants.HopperSpeed;
        indexerSpeed = Constants.IndexerSpeed;
         // in init function, set slot 0 gains
         var slot0Configs = new Slot0Configs();
         slot0Configs.kS = Constants.HopperMotorConfigsKs; // Add 0.1 V output to overcome static friction
         slot0Configs.kV = Constants.HopperMotorConfigsKv; // A velocity target of 1 rps results in 0.12 V output
         slot0Configs.kP = Constants.HopperMotorConfigsKp; // An error of 1 rps results in 0.11 V output
         slot0Configs.kI = Constants.HopperMotorConfigsKi; // no output for integrated error
         slot0Configs.kD = Constants.HopperMotorConfigsKd; // no output for error derivative

         Falcon.getConfigurator().apply(slot0Configs);


         var slot1Configs = new Slot0Configs();
         slot1Configs.kS = Constants.IndexerMotorConfigsKs; // Add 0.1 V output to overcome static friction
         slot1Configs.kV = Constants.IndexerMotorConfigsKv; // A velocity target of 1 rps results in 0.12 V output
         slot1Configs.kP = Constants.IndexerMotorConfigsKp; // An error of 1 rps results in 0.11 V output
         slot1Configs.kI = Constants.IndexerMotorConfigsKi; // no output for integrated error
         slot1Configs.kD = Constants.IndexerMotorConfigsKd; // no output for error derivative

         Kraken.getConfigurator().apply(slot1Configs); 
                   
    }

    
    public void setHopperVelocity(double rpm){ //sets the velocity for Hopper
  

       // motor.set(rpm);
        double motorRPS = (rpm * Constants.FalconGearRatio)/ 60.0;
        Falcon.setControl(m_hopperRequest.withVelocity(motorRPS));
         

    }

   
   public void setIndexerVelocity(double rpm){ //sets the velocity for Hopper
  

       // motor.set(rpm);
        double motorRPS = (rpm * Constants.KrakenGearRatio) / 60.0;
        Kraken.setControl(m_indexerRequest.withVelocity(motorRPS));
 
   }
   

    public Command feedBalls(){

        return run(()-> runMotors()); //tells to run
}


    public void runMotors(){ //uh runs

        setHopperVelocity(hopperSpeed);
        setIndexerVelocity(indexerSpeed);

    }

    public void runMotorsby2(){ //uh runs "But slower Than normal Something like that" - lingfeng

        setHopperVelocity(hopperSpeed);
        setIndexerVelocity(indexerSpeed/(Constants.SlowerIndexerSpeed));

    }
       
  public void stopMotors(){ 
    Falcon.stopMotor(); 
    Kraken.stopMotor(); 
}
    public void setHopperSpeed(double value){
        hopperSpeed = value;
    }    

    public void setIndexerSpeed(double value){
        indexerSpeed = value;
    }

            /*public void SetSlowerIndexerSpeed (double value){ might need later idk its too late for this
                slowerIndexerSpeed = indexerSpeed/(Constants.slowerIndexerSpeed);
            }
        */
        
public void setUsingState(boolean using) {
    usingState = using;
  }

  
    @Override
    public void periodic() { // the states 
        
        boolean BallsReady = !m_beambreak.get();

        if (usingState) {
    
            switch (currentState) {
         
            case IDLE:
                
            if (BallsReady) {
                
                stopMotors();
            }

            else{

                runMotorsby2();

            }
            
                 break;
        
        
            case SHOOTING:
                runMotors();
                break;

            }
        }
    }

    public void setState(State newState) {
        this.currentState = newState;
}

}
