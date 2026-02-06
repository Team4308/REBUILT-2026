public static void main(String[] args) {
    void setTarget(double degrees) {
        double targetAngle  = 0;
        if (degrees > 360) {
            targetAngle = degrees - (360 * Math.floor(degrees / 360));
            system.out.println(targetAngle)
        }
            
    }

    //setTarget(); - Needs to be Uncommented
}