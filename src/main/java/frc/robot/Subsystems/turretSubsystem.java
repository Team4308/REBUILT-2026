package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.encoders.CANCoderSwerve;

import ca.team4308.absolutelib.math.trajectories.solver.ChineseRemainderSolver;
import ca.team4308.absolutelib.math.trajectories.solver.ChineseRemainderSolver.ModularConstraint;
import ca.team4308.absolutelib.math.trajectories.solver.ChineseRemainderSolver.CRTSolution;

public class turretSubsystem extends SubsystemBase {

    private final TalonFX driveMotor;
    private final CANcoder canCoder1;
    private final CANcoder canCoder2;

    //(85/17)(40/31):1
    //(85/17)(40/33):1
    private final double CANCODER1_GEAR_RATIO = ((85.0/17.0)*(40.0/31.0))/1.0;
    private final double CANCODER2_GEAR_RATIO = ((85.0/17.0)*(40.0/33.0))/1.0;
    private final double DRIVE_MOTOR_GEAR_RATIO = (12.0/85.0);

    public double targetAngle = 0.0;
    public double degrees = 0.0;

    public turretSubsystem() {
        driveMotor = new TalonFX(0);

        canCoder1 = new CANcoder(1);
        canCoder2 = new CANcoder(2);

        //uhh the deviceids are placeholders but we can change them later, just wanted to get the code down
        //Drive Motor Config

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveMotor.getConfigurator().apply(driveConfig);
        var ccConfig = new CANcoderConfiguration();
        ccConfig.MagnetSensor.MagnetOffset = 0.0;
        
        canCoder1.getConfigurator().apply(ccConfig);
        canCoder2.getConfigurator().apply(ccConfig);
    }

    public double getTurretAngle() {

    // Get encoder rotations (0–1)
    double e1 = canCoder1.getAbsolutePosition().getValueAsDouble();
    double e2 = canCoder2.getAbsolutePosition().getValueAsDouble();

    // Convert to scaled integers for CRT
    long m1 = 31;
    long m2 = 33;

    long a1 = Math.round(e1 * 200) % m1;
    long a2 = Math.round(e2 * 200) % m2;
    long[] result = ChineseRemainderSolver.solveTwoCongruences(a1, m1, a2, m2);

    if (result == null) {
        return 0.0;
    }

    long turretScaled = result[0];
    double turretRotations = turretScaled / 200.0;
    double turretDegrees = turretRotations * 360.0;

    return MathUtil.inputModulus(turretDegrees, -180, 180);
}
    
    public boolean isAtTarget(double degrees) {
        return (Math.abs(MathUtil.inputModulus(degrees - getTurretAngle(), -180, 180)) < 1.0);
    }

    public void setTarget(double degrees) {
        targetAngle = MathUtil.inputModulus(degrees, 0, 360);
    }

    final Command moveToTarget(Supplier<Double> degrees, double timeoutMs) {
    return run(() -> {
            targetAngle = degrees.get();
            double error = MathUtil.inputModulus(targetAngle - getTurretAngle(), -180, 180);
            driveMotor.set(error * 0.01);
        }).until(() -> isAtTarget(targetAngle)).withTimeout(timeoutMs);
    }
    
     
    public Command resetTurret() {
        return run(() -> {
            targetAngle = 0.0;
            double error = MathUtil.inputModulus(targetAngle - getTurretAngle(), -180, 180);
            driveMotor.set(error * 0.01);
        }).until(() -> isAtTarget(targetAngle)).withTimeout(2000);
    }
    
    public Command aimAtHub(double degrees) {
        //find hub degrees using vision and then do it
        double hubDegrees = degrees;

        return run(() -> {
            targetAngle = hubDegrees;
            double error = MathUtil.inputModulus(targetAngle - getTurretAngle(), -180, 180);
            driveMotor.set(error * 0.01);
        }).until(() -> isAtTarget(targetAngle)).withTimeout(2000);
    }

    public Command aimAtPassingZone(double degrees) {

        double passingzoneDegrees = degrees;//aint shit in constats.java bro 😭

        return run(() -> {
            targetAngle = passingzoneDegrees;
            double error = MathUtil.inputModulus(targetAngle - getTurretAngle(), -180, 180);
            driveMotor.set(error * 0.01);
        }).until(() -> isAtTarget(targetAngle)).withTimeout(2000); 
    }
    /*
    public void setState(String state) {} son i have no clue what this is :sob:

    public void setStateBased(boolean using) {}
    */
}