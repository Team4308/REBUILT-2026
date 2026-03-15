package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems.swerve.SwerveSubsystem;
import frc.robot.Util.FuelSim;
import frc.robot.Util.SingleJointedArmSim;

public class Simulation extends SubsystemBase {
    private final HoodSubsystem m_HoodSubsystem;
    private final IndexerSubsystem m_IndexerSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;
    private final TurretSubsystem m_TurretSubsystem;
    private final SwerveSubsystem drivebase;

    private final SingleJointedArmSim m_intakeSim;
    private final SingleJointedArmSim m_hoodSim;
    private final SingleJointedArmSim m_turretSim;

    private double fuelCount = 25; // start with 25
    private double BALLS_PER_SECOND = 5;

    public Simulation(HoodSubsystem m_HoodSubsystem, IndexerSubsystem m_IndexerSubsystem,
            IntakeSubsystem m_IntakeSubsystem,
            ShooterSubsystem m_ShooterSubsystem, TurretSubsystem m_TurretSubsystem, SwerveSubsystem drivebase) {
        this.m_HoodSubsystem = m_HoodSubsystem;
        this.m_IndexerSubsystem = m_IndexerSubsystem;
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        this.m_ShooterSubsystem = m_ShooterSubsystem;
        this.m_TurretSubsystem = m_TurretSubsystem;
        this.drivebase = drivebase;

        m_intakeSim = new SingleJointedArmSim(
                DCMotor.getFalcon500(1),
                Constants.Intake.PIVOT_GEAR_RATIO,
                0.2053, // J (kg·m²)
                0.20955, // arm length (m)
                Units.degreesToRadians(0),
                Units.degreesToRadians(127),
                true, // no gravity
                6, // static friction (N·m) — intake pivot is a heavier linkage
                8, // kinetic friction (N·m)
                Units.degreesToRadians(127));

        m_hoodSim = new SingleJointedArmSim(
                DCMotor.getKrakenX44(1),
                Constants.Shooting.Hood.TOTAL_GEAR_RATIO,
                0.073752, // J (kg·m²) — small/light hood
                0.2032, // arm length (m)
                Units.degreesToRadians(0),
                Units.degreesToRadians(45),
                false, // gravity
                7, // static friction
                0.5, // kinetic friction
                Units.degreesToRadians(0));

        m_turretSim = new SingleJointedArmSim(
                DCMotor.getKrakenX44(1),
                1. / Constants.Shooting.Turret.GEAR_RATIO_MOTOR,
                0.2, // J (kg·m²) — heaviest of the three
                0.1, // arm length unused (turret rotates about center, not an arm)
                Units.degreesToRadians(90),
                Units.degreesToRadians(500),
                false, // no gravity
                5, // static friction (N·m) — large rotating mass, more bearing friction
                4, // kinetic friction (N·m)
                Units.degreesToRadians(360));

        m_turretSim.update(0);

        m_IntakeSubsystem.setSimSupplier(() -> Math.toDegrees(m_intakeSim.getAngleRads()));
        m_HoodSubsystem.setSimSupplier(() -> Math.toDegrees(m_hoodSim.getAngleRads()));
        m_TurretSubsystem.setSimSupplier(() -> Math.toDegrees(m_turretSim.getAngleRads()),
                () -> DoubleUtils.clamp(
                        Math.toDegrees(m_turretSim.getAngleRads()) * Constants.Shooting.Turret.GEAR_RATIO_1 / 360, 0,
                        1),
                () -> DoubleUtils.clamp(
                        Math.toDegrees(m_turretSim.getAngleRads()) * Constants.Shooting.Turret.GEAR_RATIO_2 / 360, 0,
                        1));

        initFuelSim();
    }

    private void incrementFuel() {
        fuelCount = Math.min(fuelCount + 1, Constants.Simulation.MAX_FUEL);
    }

    private void initFuelSim() {
        FuelSim.getInstance(); // gets singleton instance of FuelSim
        FuelSim.getInstance().spawnStartingFuel(); // spawns fuel in the depots and neutral zone

        // Register a robot for collision with fuel
        FuelSim.getInstance().registerRobot(
                Constants.BOT_WIDTH, // from left to right
                Constants.BOT_WIDTH, // from front to back
                Constants.BUMPER_HEIGHT, // from floor to top of bumpers
                drivebase::getPose, // Supplier<Pose2d> of robot pose
                drivebase::getFieldVelocity); // Supplier<ChassisSpeeds> of field-centric chassis speeds

        FuelSim.getInstance().registerIntake(Constants.Simulation.FuelSim.xMin, Constants.Simulation.FuelSim.xMax,
                Constants.Simulation.FuelSim.yMin, Constants.Simulation.FuelSim.yMax,
                () -> m_IntakeSubsystem.getIntakeAngle() < 10 && fuelCount <= Constants.Simulation.MAX_FUEL,
                () -> incrementFuel());

        FuelSim.getInstance().setSubticks(5); // sets the number of physics iterations to perform per 20ms loop.

        FuelSim.getInstance().start();
    }

    public void launchFuel() {
        if (fuelCount == 0)
            return;
        fuelCount--;

        FuelSim.getInstance().launchFuel(
                m_ShooterSubsystem.getTargetRPM() * 3.14 * 4 * 0.0254 / 60 * 0.7, // compression slow down and rpm
                                                                                  // slow down
                90 - m_HoodSubsystem.getHoodAngle(),
                m_TurretSubsystem.getAngleWrapped() + 180,
                Units.inchesToMeters(10));

    }

    double intervalRemaining = 0;

    @Override
    public void periodic() {
        m_intakeSim.setInputVoltage(m_IntakeSubsystem.getVoltage());
        m_hoodSim.setInputVoltage(m_HoodSubsystem.getVoltage());
        m_turretSim.setInputVoltage(m_TurretSubsystem.getVoltage());

        m_intakeSim.update(0.020);
        m_hoodSim.update(0.020);
        m_turretSim.update(0.020);

        if (m_IndexerSubsystem.getTargetBallTunnelVelocity() > 10) {
            if (intervalRemaining <= 0) {
                launchFuel();
                intervalRemaining = 1 / BALLS_PER_SECOND; // 5bps
            }
            intervalRemaining -= 0.02;
        }

        Logger.recordOutput("Simulation/Fuel Count", fuelCount);
        Logger.recordOutput("Simulation/Red Hub Count", FuelSim.Hub.RED_HUB.getScore());
        Logger.recordOutput("Simulation/Blue Hub Score", FuelSim.Hub.BLUE_HUB.getScore());
    }
}
