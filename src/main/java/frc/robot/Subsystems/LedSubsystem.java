package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

    private static final int LED_PORT = 1;
    private static final int LED_LENGTH = 58;
    private static final double BRIGHTNESS = 0.5;

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private int hueOffset = 0;
    private double pulseValue = 0;
    private double pulseDirection = 1;

    public LedSubsystem() {
        led = new AddressableLED(LED_PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(LED_LENGTH);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        // Pulse brightness up and down
        pulseValue += 0.02 * pulseDirection;
        if (pulseValue >= 1.0) {
            pulseValue = 1.0;
            pulseDirection = -1;
        }
        if (pulseValue <= 0.0) {
            pulseValue = 0.0;
            pulseDirection = 1;
        }

        double brightness = BRIGHTNESS * pulseValue;

        for (int i = 0; i < LED_LENGTH; i++) {
            int hue = (hueOffset + (i * 180 / LED_LENGTH)) % 180;
            buffer.setHSV(i, hue, 255, (int) (brightness * 255));
        }

        hueOffset = (hueOffset + 2) % 180;
        led.setData(buffer);
    }
}