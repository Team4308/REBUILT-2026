package frc.robot.Subsystems;

import java.util.Map;

import ca.team4308.absolutelib.leds.AddressableLEDBufferView;
import ca.team4308.absolutelib.leds.LEDPattern;
import ca.team4308.absolutelib.leds.Patterns;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.Leds;

public class LedSubsystem extends AbsoluteSubsystem {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView view; // Front
    private final AddressableLEDBufferView viewBack; // Back
    private final AddressableLEDBufferView viewLeft; // Left
    private final AddressableLEDBufferView viewRight; // Right
    private final AddressableLEDBufferView underGlow; // Underneath the bot (In brain pan?)
    private Map<Integer, AddressableLEDBufferView> allViews = new java.util.HashMap<>();
    private Pose2d[] DriverSationCords = {
        // Blue Alliance driver station locations 
        new Pose2d(0, 7, new edu.wpi.first.math.geometry.Rotation2d()),
        new Pose2d(0, 4, new edu.wpi.first.math.geometry.Rotation2d()),
        new Pose2d(0, 1, new edu.wpi.first.math.geometry.Rotation2d()),
    };
    

    private LEDPattern currentPattern;
    private LEDPattern driverPattern; /** Data for the driver **/
    private String currentPatternName = "Idle";
    private String driverPatternName = "Idle";
    private static final int LED_PORT = Leds.LED_PORT;
    private static final int LED_LENGTH = Leds.LED_LENGTH;
    private static final double BRIGHTNESS = 0.55; // 0.0 = off, 1.0 = full brightness

    public LedSubsystem() {
        super();
        led = new AddressableLED(LED_PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(LED_LENGTH);
        led.setData(buffer);
        led.start();
        view = new AddressableLEDBufferView(buffer, Leds.startIndexes[0], Leds.startIndexes[1] - Leds.startIndexes[0]);
        viewBack = new AddressableLEDBufferView(buffer, Leds.startIndexes[1], Leds.startIndexes[2] - Leds.startIndexes[1]);
        viewLeft = new AddressableLEDBufferView(buffer, Leds.startIndexes[2], Leds.startIndexes[3] - Leds.startIndexes[2]);
        viewRight = new AddressableLEDBufferView(buffer, Leds.startIndexes[3], Leds.startIndexes[4] - Leds.startIndexes[3]);
        underGlow = new AddressableLEDBufferView(buffer, Leds.startIndexes[4], LED_LENGTH - Leds.startIndexes[4]);
        allViews.put(Leds.viewAngles[0], view);
        allViews.put(Leds.viewAngles[1], viewBack);
        allViews.put(Leds.viewAngles[2], viewLeft);
        allViews.put(Leds.viewAngles[3], viewRight);

        currentPattern = Patterns.scrollingIdle(Color.kDarkRed, 1);
        driverPattern = Patterns.scrollingIdle(Color.kDarkRed, 1);
    }


    public AddressableLEDBufferView getViewFacingDriver(SwerveDriveOdometry3d odometry) {
        double robotX = odometry.getPoseMeters().getX();
        double robotY = odometry.getPoseMeters().getY();
        double robotAngle = odometry.getPoseMeters().getRotation().toRotation2d().getDegrees();
        
        double dsX = 0.0;
        double dsY = 0.0;

        var location = DriverStation.getLocation();
        if (location.isPresent() && location.getAsInt() >= 1 && location.getAsInt() <= 3) {
            Pose2d dsPose = DriverSationCords[location.getAsInt() - 1];
            dsX = dsPose.getX();
            dsY = dsPose.getY();
        }

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            dsX = 16.541;
        }

        double driverStationAngle = Math.toDegrees(Math.atan2(dsY - robotY, dsX - robotX));

        double angleToDriver = (driverStationAngle - robotAngle) % 360.0;
        if (angleToDriver < 0) {
            angleToDriver += 360.0;
        }

        int nearestViewAngle = (int) Math.round(angleToDriver / 90.0) * 90;
        if (nearestViewAngle == 360) {
            nearestViewAngle = 0;
        }

        return allViews.get(nearestViewAngle);
    }

    @Override
    public void periodic() {
        if (currentPattern != null) {
            for (AddressableLEDBufferView view : allViews.values()) {
                // Uncomment code when we have swerve added
                // if (view == getViewFacingDriver(StateManager.getInstance().getSwerveOdometry())) {
                //     driverPattern.applyTo(view);
                // }
                currentPattern.applyTo(view);
            }
            Patterns.defaultScrollingIdle().applyTo(underGlow);
        }
        // Dim the entire buffer to the desired brightness
        for (int i = 0; i < buffer.getLength(); i++) {
            Color c = buffer.getLED(i);
            buffer.setLED(i, new Color(c.red * BRIGHTNESS, c.green * BRIGHTNESS, c.blue * BRIGHTNESS));
        }
        led.setData(buffer);
    }

    public void setIdle() {
        currentPattern = Patterns.scrollingIdle(Color.lerpRGB(Color.kDodgerBlue, Color.kBlue, 0.5), 1);
        currentPatternName = "Idle";

        driverPattern = Patterns.scrollingIdle(Color.lerpRGB(Color.kDodgerBlue, Color.kBlue, 0.5), 1);
        driverPatternName = "Idle";

    }

    /**
     * Sets the Drivers LED pattern to error      
     */
    public void setError() {
        driverPattern = Patterns.error();
        driverPatternName = "Error";
    }

        /**
     * Sets the Drivers LED pattern to success      
     */
    public void setSuccess() {
        driverPattern = Patterns.success();
        driverPatternName = "Success";
    }
    /**
     * Sets the Drivers LED pattern to warning      
     */
    public void setWarning() {
        driverPattern = Patterns.warning();
        driverPatternName = "Warning";
    }
    /**
     * Sets the Drivers LED pattern to loading      
     * @param speed the speed of the loading pattern, where 1.0 is normal speed
     */
    public void setWarning(double speed) {
        driverPattern = Patterns.createBreathingPattern(Color.kYellow, speed);
        driverPatternName = "Warning";
    }




    public void setChasingDot(Color color) {
        currentPattern = Patterns.chasingDot(color);
        currentPatternName = "Chasing Dot";
    }

    // public void applyStatePattern(StateManager.RobotState state) {
    //     String name = state.name();
    //     if (name.equals(currentPatternName)) {
    //         return;
    //     }

    //     switch (state) {
    //         case Home :
    //             currentPattern = Patterns.getAllianceBreathing(2.0);
    //             break;

    //         case ActiveTeleopAllianceZone, ActiveTeleopOpponentZone, ActiveTeleopNeutralZone:
    //             currentPattern = Patterns.altF4();
    //             break;

    //         case InactiveTeleopAllianceZone,InactiveTeleopNeutralZone, InactiveTeleopOpponentZone:
    //             currentPattern = Patterns.defaultScrollingIdle();
    //             break;

    //         case EndgameTeleopAllianceZone,EndgameTeleopNeutralZone, EndgameTeleopOpponentZone:
    //                 currentPattern = Patterns.createBlinkingPattern(Color.kRed, DriverStation.getMatchTime() / 30.0);
    //         break;

    //         default:
    //             currentPattern = Patterns.defaultScrollingIdle();
    //             break;
    //     }

    //     currentPatternName = name;
    // }

    public String getCurrentPatternName() {
        return currentPatternName + ", " + driverPatternName;
    }

    @Override
    public Sendable log() {
        return null;
    }
}