package frc.robot;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IndexerSubsystem extends SubsystemBase{
   
    private State currentState = State.IDLE;
    
    
    public enum State { //the diff states 
        IDLE,
        SHOOTING
    }
    
    final public double RollerSpeed = Constants.HopperSpeed; //takes the speed set in constants
    TalonFX Falcon = new TalonFX(Constants.HopperMotorId);
    TalonFX Kraken = new TalonFX(Constants.IndexerMotorId);
    
    private final VelocityVoltage m_hopperRequest = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_indexerRequest = new VelocityVoltage(0).withSlot(0);

    
    public IndexerSubsystem(){
         
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

        return run(()-> RunMotors()); //tells to run
}


    public void RunMotors(){ //uh runs

        setHopperVelocity(Constants.HopperSpeed);
        setIndexerVelocity(Constants.IndexerSpeed);

    }
       
  public void StopMotors(){ 
    Falcon.stopMotor(); 
    Kraken.stopMotor(); 
}
    

    @Override
    public void periodic() { // the states 
        
      switch (currentState) {
         
        case IDLE:
            StopMotors();
             break;
        
        
        case SHOOTING:
            RunMotors();
             break;

        
        }
    }

    public void setState(State newState) {
        this.currentState = newState;
}
}
