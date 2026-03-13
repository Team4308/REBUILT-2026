package frc.robot.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Represents a simulated single jointed arm mechanism. */
public class SingleJointedArmSim extends LinearSystemSim<N2, N1, N2> {
    // The gearbox for the arm.
    private final DCMotor m_gearbox;

    // The gearing between the motors and the output.
    private final double m_gearing;

    // The length of the arm.
    private final double m_armLenMeters;

    // The minimum angle that the arm is capable of.
    private final double m_minAngle;

    // The maximum angle that the arm is capable of.
    private final double m_maxAngle;

    // Whether the simulator should simulate gravity.
    private final boolean m_simulateGravity;

    // The moment of inertia of the arm in kg·m². Used to convert friction torques
    // to accelerations.
    private final double m_jKgMetersSquared;

    // The torque (N·m) required to break static friction and begin moving.
    private final double m_staticFrictionTorque;

    // The torque (N·m) opposing motion while the arm is already moving.
    private final double m_kineticFrictionTorque;

    /**
     * Creates a simulated arm mechanism with friction.
     *
     * @param plant                 The linear system that represents the arm. This
     *                              system can be created with {@link
     *                              edu.wpi.first.math.system.plant.LinearSystemId#createSingleJointedArmSystem(DCMotor,
     *                              double, double)}.
     * @param gearbox               The type of and number of motors in the arm
     *                              gearbox.
     * @param gearing               The gearing of the arm (numbers greater than 1
     *                              represent reductions).
     * @param jKgMetersSquared      The moment of inertia of the arm in kg·m².
     * @param armLengthMeters       The length of the arm.
     * @param minAngleRads          The minimum angle that the arm is capable of.
     * @param maxAngleRads          The maximum angle that the arm is capable of.
     * @param simulateGravity       Whether gravity should be simulated or not.
     * @param staticFrictionTorque  The torque (N·m) required to break static
     *                              friction. Set to 0 to
     *                              disable.
     * @param kineticFrictionTorque The torque (N·m) opposing motion while the arm
     *                              is moving. Should
     *                              be less than or equal to staticFrictionTorque.
     *                              Set to 0 to disable.
     * @param startingAngleRads     The initial position of the Arm simulation in
     *                              radians.
     * @param measurementStdDevs    The standard deviations of the measurements. Can
     *                              be omitted if no
     *                              noise is desired. If present must have 1 element
     *                              for position.
     */
    @SuppressWarnings("this-escape")
    public SingleJointedArmSim(
            LinearSystem<N2, N1, N2> plant,
            DCMotor gearbox,
            double gearing,
            double jKgMetersSquared,
            double armLengthMeters,
            double minAngleRads,
            double maxAngleRads,
            boolean simulateGravity,
            double staticFrictionTorque,
            double kineticFrictionTorque,
            double startingAngleRads,
            double... measurementStdDevs) {
        super(plant, measurementStdDevs);
        m_gearbox = gearbox;
        m_gearing = gearing;
        m_jKgMetersSquared = jKgMetersSquared;
        m_armLenMeters = armLengthMeters;
        m_minAngle = minAngleRads;
        m_maxAngle = maxAngleRads;
        m_simulateGravity = simulateGravity;
        m_staticFrictionTorque = Math.abs(staticFrictionTorque);
        m_kineticFrictionTorque = Math.abs(kineticFrictionTorque);

        setState(startingAngleRads, 0.0);
    }

    /**
     * Creates a simulated arm mechanism with friction.
     *
     * @param gearbox               The type of and number of motors in the arm
     *                              gearbox.
     * @param gearing               The gearing of the arm (numbers greater than 1
     *                              represent reductions).
     * @param jKgMetersSquared      The moment of inertia of the arm; can be
     *                              calculated from CAD software.
     * @param armLengthMeters       The length of the arm.
     * @param minAngleRads          The minimum angle that the arm is capable of.
     * @param maxAngleRads          The maximum angle that the arm is capable of.
     * @param simulateGravity       Whether gravity should be simulated or not.
     * @param staticFrictionTorque  The torque (N·m) required to break static
     *                              friction.
     * @param kineticFrictionTorque The torque (N·m) opposing motion while moving.
     * @param startingAngleRads     The initial position of the Arm simulation in
     *                              radians.
     * @param measurementStdDevs    The standard deviations of the measurements. Can
     *                              be omitted if no
     *                              noise is desired. If present must have 1 element
     *                              for position.
     */
    public SingleJointedArmSim(
            DCMotor gearbox,
            double gearing,
            double jKgMetersSquared,
            double armLengthMeters,
            double minAngleRads,
            double maxAngleRads,
            boolean simulateGravity,
            double staticFrictionTorque,
            double kineticFrictionTorque,
            double startingAngleRads,
            double... measurementStdDevs) {
        this(
                LinearSystemId.createSingleJointedArmSystem(gearbox, jKgMetersSquared, gearing),
                gearbox,
                gearing,
                jKgMetersSquared,
                armLengthMeters,
                minAngleRads,
                maxAngleRads,
                simulateGravity,
                staticFrictionTorque,
                kineticFrictionTorque,
                startingAngleRads,
                measurementStdDevs);
    }

    /**
     * Sets the arm's state. The new angle will be limited between the minimum and
     * maximum allowed
     * limits.
     *
     * @param angleRadians      The new angle in radians.
     * @param velocityRadPerSec The new angular velocity in radians per second.
     */
    public final void setState(double angleRadians, double velocityRadPerSec) {
        setState(
                VecBuilder.fill(MathUtil.clamp(angleRadians, m_minAngle, m_maxAngle), velocityRadPerSec));
    }

    /**
     * Returns whether the arm would hit the lower limit.
     *
     * @param currentAngleRads The current arm angle.
     * @return Whether the arm would hit the lower limit.
     */
    public boolean wouldHitLowerLimit(double currentAngleRads) {
        return currentAngleRads <= this.m_minAngle;
    }

    /**
     * Returns whether the arm would hit the upper limit.
     *
     * @param currentAngleRads The current arm angle.
     * @return Whether the arm would hit the upper limit.
     */
    public boolean wouldHitUpperLimit(double currentAngleRads) {
        return currentAngleRads >= this.m_maxAngle;
    }

    /**
     * Returns whether the arm has hit the lower limit.
     *
     * @return Whether the arm has hit the lower limit.
     */
    public boolean hasHitLowerLimit() {
        return wouldHitLowerLimit(getAngleRads());
    }

    /**
     * Returns whether the arm has hit the upper limit.
     *
     * @return Whether the arm has hit the upper limit.
     */
    public boolean hasHitUpperLimit() {
        return wouldHitUpperLimit(getAngleRads());
    }

    /**
     * Returns the current arm angle.
     *
     * @return The current arm angle.
     */
    public double getAngleRads() {
        return getOutput(0);
    }

    /**
     * Returns the current arm velocity.
     *
     * @return The current arm velocity.
     */
    public double getVelocityRadPerSec() {
        return getOutput(1);
    }

    /**
     * Returns the arm current draw.
     *
     * @return The arm current draw.
     */
    public double getCurrentDrawAmps() {
        // Reductions are greater than 1, so a reduction of 10:1 means the motor spins
        // 10x faster than the output.
        var motorVelocity = m_x.get(1, 0) * m_gearing;
        return m_gearbox.getCurrent(motorVelocity, m_u.get(0, 0)) * Math.signum(m_u.get(0, 0));
    }

    /**
     * Sets the input voltage for the arm.
     *
     * @param volts The input voltage.
     */
    public void setInputVoltage(double volts) {
        setInput(volts);
        clampInput(RobotController.getBatteryVoltage());
    }

    /**
     * Calculates a rough estimate of the moment of inertia of an arm given its
     * length and mass.
     *
     * @param lengthMeters The length of the arm.
     * @param massKg       The mass of the arm.
     * @return The calculated moment of inertia.
     */
    public static double estimateMOI(double lengthMeters, double massKg) {
        return 1.0 / 3.0 * massKg * lengthMeters * lengthMeters;
    }

    /**
     * Updates the state of the arm.
     *
     * <p>
     * Friction model:
     * <ul>
     * <li>When the arm is stationary (|ω| &lt; threshold), static friction opposes
     * the net driving
     * torque. If the net torque is below the static friction threshold, the arm
     * stays put. If
     * it exceeds the threshold, the arm breaks free and kinetic friction takes
     * over.
     * <li>While the arm is moving (|ω| ≥ threshold), kinetic friction opposes the
     * direction of
     * velocity. A sign-flip check after integration zeroes the velocity if friction
     * would have
     * stalled the arm mid-step.
     * </ul>
     *
     * @param currentXhat The current state estimate.
     * @param u           The system inputs (voltage).
     * @param dtSeconds   The time difference between controller updates.
     */
    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        // The torque on the arm is given by τ = F⋅r, where F is the force applied by
        // gravity and r the distance from pivot to center of mass. Recall from
        // dynamics that the sum of torques for a rigid body is τ = J⋅α, where τ is
        // torque on the arm, J is the mass-moment of inertia about the pivot axis,
        // and α is the angular acceleration in rad/s². Rearranging yields: α = F⋅r/J
        //
        // We substitute in F = m⋅g⋅cos(θ), where θ is the angle from horizontal:
        //
        // α = (m⋅g⋅cos(θ))⋅r/J
        //
        // Multiply RHS by cos(θ) to account for the arm angle. Further, we know the
        // arm mass-moment of inertia J of our arm is given by J=1/3 mL², modeled as a
        // rod rotating about its end, where L is the overall rod length. The mass
        // distribution is assumed to be uniform. Substitute r=L/2 to find:
        //
        // α = (m⋅g⋅cos(θ))⋅r/(1/3 mL²)
        // α = (m⋅g⋅cos(θ))⋅(L/2)/(1/3 mL²)
        // α = 3/2⋅g⋅cos(θ)/L
        //
        // This acceleration is next added to the linear system dynamics ẋ=Ax+Bu
        //
        // f(x, u) = Ax + Bu + [0 α]ᵀ
        // f(x, u) = Ax + Bu + [0 3/2⋅g⋅cos(θ)/L]ᵀ
        //
        // Friction is then layered on top as an additional angular acceleration term.
        // Static friction exactly cancels the net driving acceleration up to the
        // stiction threshold; above it, kinetic friction opposes motion direction.

        // Pre-compute friction acceleration magnitudes from torque values.
        final double staticAlpha = m_staticFrictionTorque / m_jKgMetersSquared;
        final double kineticAlpha = m_kineticFrictionTorque / m_jKgMetersSquared;

        // Velocity sign at the start of this timestep — used later for stall detection.
        final double omegaBefore = currentXhat.get(1, 0);

        Matrix<N2, N1> updatedXhat = NumericalIntegration.rkdp(
                (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
                    // Base dynamics: ẋ = Ax + Bu
                    Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));

                    // Add gravity acceleration if enabled.
                    if (m_simulateGravity) {
                        double alphaGrav = 3.0 / 2.0 * -9.8 * Math.cos(x.get(0, 0)) / m_armLenMeters;
                        xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
                    }

                    double omega = x.get(1, 0);

                    // Net angular acceleration from motor + gravity (before friction).
                    double netAlpha = xdot.get(1, 0);

                    double frictionAlpha;
                    if (Math.abs(omega) < 1e-6) {
                        // --- Static friction regime ---
                        // The arm is essentially stationary. Static friction opposes whatever
                        // is trying to move it. If the net driving acceleration is within the
                        // stiction band, friction cancels it completely and the arm stays still.
                        if (Math.abs(netAlpha) <= staticAlpha) {
                            frictionAlpha = -netAlpha; // perfect cancellation — arm holds position
                        } else {
                            // Net force exceeds stiction: arm starts to move.
                            // Apply kinetic friction opposing the direction of impending motion.
                            frictionAlpha = -Math.signum(netAlpha) * kineticAlpha;
                        }
                    } else {
                        // --- Kinetic friction regime ---
                        // The arm is moving; friction always opposes the current velocity.
                        frictionAlpha = -Math.signum(omega) * kineticAlpha;
                    }

                    return xdot.plus(VecBuilder.fill(0, frictionAlpha));
                },
                currentXhat,
                u,
                dtSeconds);

        // Post-integration stall check:
        // If kinetic friction caused the velocity to reverse sign during this step,
        // the arm actually came to rest somewhere mid-step. Clamp velocity to zero
        // to prevent the integrator from overshooting through zero.
        double omegaAfter = updatedXhat.get(1, 0);
        if (Math.signum(omegaAfter) != Math.signum(omegaBefore) && Math.abs(omegaBefore) > 1e-6) {
            updatedXhat = VecBuilder.fill(updatedXhat.get(0, 0), 0.0);
        }

        // Hard stops — zero velocity at each physical limit.
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(m_minAngle, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(m_maxAngle, 0);
        }
        return updatedXhat;
    }
}