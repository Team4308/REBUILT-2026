package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import com.ctre.phoenix6.hardware.TalonFX;

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class turretSubsystem extends SubsystemBase {

    private final TalonFX turretMotor;

    public turretSubsystem() {
        turretMotor = new TalonFX(0);
    }

    public double getTurretAngle() {
        return 0.0;
        //placeholder for finding the actual turret angle
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

    public void resetTurret(double degrees) {
        degrees = 0.0;
        double error = degrees - getTurretAngle();
        turretMotor.set(error * 0.01);
    }

    //setTarget(); - Needs to be Uncommented
    // boolean isAtTarget() {} // returns whether the turret degree is within x degrees of the target. (x to be put in constants.java)

    // Command moveToTarget(Supplier<Double> degrees) {} // runs a command that moves the turret to the target. It must end when the turret is at the correct position

    // Command moveToTarget(Supplier<Double> degrees, double timeoutMs) {} // similar to moveToTarget, but if it reaches the timeout (milliseconds) first, it must end no matter what.

    // void resetTurret() {} // moves back to starting position

    // Command resetTurret() {} // moves back to starting position, until it is at target

    // void aimAtHub() {} // auto targets to hub - talk to nicholas on how to do this.
    // Command aimAtHub() {} // same but its an command - it should not end, but rather run until interrupted

    // void aimAtPassingZone() {} // aims towards the area we will pass too - specific location will be in strategy
    // Command aimAtPassingZone() {} // same thing, it will not end until interrupted

    // void setState(String state) {} // sets the current state

    // void setStateBased(boolean using) {} // turns on/off the state manager


}