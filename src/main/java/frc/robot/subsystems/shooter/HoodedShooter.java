package frc.robot.subsystems.shooter;

import ca.team4308.absolutelib.math.trajectories.ShotInput;
import ca.team4308.absolutelib.math.trajectories.TrajectorySolver;
import ca.team4308.absolutelib.subsystems.Pivot;
import ca.team4308.absolutelib.subsystems.simulation.PivotSimulation;
import ca.team4308.absolutelib.wrapper.EncoderWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper;
import ca.team4308.absolutelib.wrapper.MotorWrapper.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;

public class HoodedShooter {
    private final Pivot m_HoodedShooter;

    public HoodedShooter() {
        // Do not call super; this class composes a Pivot instance instead of
        // extending it. The Pivot must be constructed via Pivot.Config as used
        // below to avoid the IllegalStateException raised by the default
        // constructor.
        MotorWrapper leader = new MotorWrapper(MotorType.TALONFX, 20);
        Pivot.Config pivotConfig = new Pivot.Config()
                .withLeader(leader)
                .withEncoder(EncoderWrapper.canCoder(21, 1.0)) 
                .gear(50.0)
                .limits(5, 80)
                .pid(55.0, 0.0, 0.1)
                .ff(0.5, 0.1, 0.1, 0.1)
                .useSmartMotion(false) 
                .enableSimulation(true)
                .withSimulation(
                        new PivotSimulation.Config()
                                .gearbox(DCMotor.getNEO(1), 1)
                                .gearRatio(50.0)
                                .armLength(0.5)
                                .armMass(3.0)
                                .limits(Math.toRadians(5), Math.toRadians(80))
                                .startAngle(0.0)
                );

        this.m_HoodedShooter = new Pivot(pivotConfig);
        this.m_HoodedShooter.initialize(); // Ensures simulation setup runs redundant since this should be auto called

        // Pitch Slover
        TrajectorySolver solver = TrajectorySolver.forGame2026();

    }

    public ShotInput SloveInput(double x, double y, double z ) {
              return ShotInput.builder()
                .shooterPositionMeters(x, y, z) 
                .targetPositionMeters(4.0, 3.0, 4.5) // HUB (Blue Side)
                .shotPreference(ShotInput.ShotPreference.AUTO)
                .build();
    }


    
    public void periodic() {
        m_HoodedShooter.periodic();
    }

    /**
     * Expose the internal Pivot (which implements Subsystem) so commands can add
     * requirements on it.
     */
    public Pivot getPivot() {
        return m_HoodedShooter;
    }

    

    public Command setAngle(double degrees) {
        return m_HoodedShooter.setPosition(degrees);
    }



}
