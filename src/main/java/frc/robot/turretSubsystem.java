package frc.robot;

public static void main(String[] args) {
    
    private final TalonFX motor1; 

    double getTurretAngle() {}   // returns rotation in degrees from 0-360

    void setTarget(double degrees) {
        double targetAngle  = 0;
        if (degrees > 360) {
            targetAngle = degrees - (360 * Math.floor(degrees / 360));
            system.out.println(targetAngle)
        }    
    }
    //setTarget(); - Needs to be Uncommented
    boolean isAtTarget() {} // returns whether the turret degree is within x degrees of the target. (x to be put in constants.java)

    Command moveToTarget(Supplier<Double> degrees) {} // runs a command that moves the turret to the target. It must end when the turret is at the correct position

    Command moveToTarget(Supplier<Double> degrees, double timeoutMs) {} // similar to moveToTarget, but if it reaches the timeout (milliseconds) first, it must end no matter what.

    void resetTurret() {} // moves back to starting position

    Command resetTurret() {} // moves back to starting position, until it is at target

    void aimAtHub() {} // auto targets to hub - talk to nicholas on how to do this.
    Command aimAtHub() {} // same but its an command - it should not end, but rather run until interrupted

    void aimAtPassingZone() {} // aims towards the area we will pass too - specific location will be in strategy
    Command aimAtPassingZone() {} // same thing, it will not end until interrupted

    void setState(String state) {} // sets the current state

    void setStateBased(boolean using) {} // turns on/off the state manager

        
}