package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class turretSubsystem extends SubsystemBase {

    private final TalonFX turretMotor;

    private final double CANCODER1_GEAR_RATIO = 85/31;
    private final double CANCODER2_GEAR_RATIO = 85/33;
    private final double DRIVE_MOTOR_GEAR_RATIO = 12/85;

    public double targetAngle = 0.0;
    private boolean isAtTarget = false;

    public turretSubsystem() {
        turretMotor = new TalonFX(0);

    }

    public double getTurretAngle() {
        return 0.0;
    }

    public boolean isAtTarget(double degrees) {
        return Math.abs(MathUtil.inputModulus(degrees - getTurretAngle(), -180, 180)) < 1.0;
    }

    public void setTarget(double degrees) {
        double targetAngle  = 0;
        if (degrees > 360) {
            targetAngle = degrees - (360 * Math.floor(degrees / 360));
            System.out.println(targetAngle);//should be return targetAngle, no?
        } 
    }

    public Command moveToTarget(Supplier<Double> degrees) {
        return run(() -> {
            double currentTarget = degrees.get();
            double error = currentTarget - getTurretAngle();
            turretMotor.set(error * 0.01);
        }).until(this::isAtTarget);
    }
    /* 
    public Command moveToTarget(Supplier<Double> degrees, double timeoutMs) {

    }

    public void resetTurret() {}
    public Command resetTurret() {}

    public void aimAtHub() {}
    public Command aimAtHub() {}

    public void aimAtPassingZone() {}
    public Command aimAtPassingZone() {}

    public void setState(String state) {}

    public void setStateBased(boolean using) {}
    */
}