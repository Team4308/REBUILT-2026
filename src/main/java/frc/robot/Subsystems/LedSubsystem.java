package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;

import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.Leds;
import frc.robot.Util.AddressableLEDBufferView;
import frc.robot.Util.LEDPattern;
import frc.robot.Util.Patterns;

public class LedSubsystem extends AbsoluteSubsystem {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView hopperLeft;
    private final AddressableLEDBufferView hopperRight;
    private final AddressableLEDBufferView shooterLeft;
    private final AddressableLEDBufferView shooterRight;

    private List<AddressableLEDBufferView> bufferList = new ArrayList<>();

    private LEDPattern currentPattern;
    private static final int LED_PORT = Leds.LED_PORT;
    private static final int LED_LENGTH = Leds.LED_LENGTH;
    private static final double BRIGHTNESS = 0.55;

    // Tune this to whatever current (amps) reliably indicates the intake is running
    private static final double INTAKE_CURRENT_THRESHOLD = 5.0;

    // How long (ms) to blink the LEDs off when the beambreak triggers
    private static final long BEAMBREAK_BLINK_DURATION_MS = 150;

    private boolean prevBeambreak = false;
    private long beambreakBlinkStartMs = -1;

    private final IntakeSubsystem m_IntakeSubsystem;
    private final IndexerSubsystem m_IndexerSubsystem;

    public LedSubsystem(IntakeSubsystem m_IntakeSubsystem, IndexerSubsystem m_IndexerSubsystem) {
        super();
        led = new AddressableLED(LED_PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(LED_LENGTH);
        led.setData(buffer);
        led.start();

        hopperLeft = new AddressableLEDBufferView(buffer, Leds.startIndexes[0],
                Leds.startIndexes[1] - Leds.startIndexes[0]);
        hopperRight = new AddressableLEDBufferView(buffer, Leds.startIndexes[1],
                Leds.startIndexes[2] - Leds.startIndexes[1]);
        shooterLeft = new AddressableLEDBufferView(buffer, Leds.startIndexes[2],
                Leds.startIndexes[3] - Leds.startIndexes[2]);
        shooterRight = new AddressableLEDBufferView(buffer, Leds.startIndexes[3],
                Leds.startIndexes[4] - Leds.startIndexes[3]);

        bufferList.add(hopperLeft);
        bufferList.add(hopperRight);
        bufferList.add(shooterLeft);
        bufferList.add(shooterRight);

        currentPattern = Patterns.scrollingIdle(Color.kDarkRed, 1);

        this.m_IndexerSubsystem = m_IndexerSubsystem;
        this.m_IntakeSubsystem = m_IntakeSubsystem;
    }

    private LEDPattern getIdlePattern() {
        if (DriverStation.getAlliance().get().equals(Alliance.Red))
            return Patterns.scrollingIdle(Color.lerpRGB(Color.kDarkRed, Color.kRed, 0.5), 1);
        return Patterns.scrollingIdle(Color.lerpRGB(Color.kDarkBlue, Color.kBlue, 0.5), 1);
    }

    /** Periodic yellow blink — one flash every 0.5 s */
    private LEDPattern getIntakingPattern() {
        return Patterns.createBlinkingPattern(Color.kYellow, 0.5);
    }

    /** Solid green used for shooting / intake+shooting base state */
    private LEDPattern getSolidGreenPattern() {
        return Patterns.createSolidPattern(Color.kGreen);
    }

    /** All-off pattern used for the beambreak blink-off window */
    private LEDPattern getOffPattern() {
        return Patterns.createSolidPattern(Color.kBlack);
    }

    /**
     * Update LEDs and apply hopper-fill mask.
     * 
     * @param hopperFillCount number of hopper LEDs (starting from the leftmost
     *                        hopper LED) to keep lit. Remaining hopper LEDs will be
     *                        forced off.
     */
    public void updateLeds(int hopperFillCount) {
        for (AddressableLEDBufferView view : bufferList) {
            currentPattern.applyTo(view);
        }

        // Mask out hopper LEDs beyond the requested fill count so only the first
        // `hopperFillCount` LEDs (starting from the left hopper end) remain visible.
        int leftStart = Leds.startIndexes[0];
        int leftEnd = Leds.startIndexes[1];
        int rightStart = Leds.startIndexes[1];
        int rightEnd = Leds.startIndexes[2];

        int leftLen = leftEnd - leftStart;
        int rightLen = rightEnd - rightStart;
        int totalHopper = leftLen + rightLen;

        int fill = Math.max(0, Math.min(hopperFillCount, totalHopper));

        for (int i = 0; i < totalHopper; i++) {
            if (i >= fill) {
                int globalIndex = (i < leftLen) ? (leftStart + i) : (rightStart + (i - leftLen));
                buffer.setLED(globalIndex, Color.kBlack);
            }
        }

        // Apply global brightness scalar
        for (int i = 0; i < buffer.getLength(); i++) {
            Color c = buffer.getLED(i);
            buffer.setLED(i, new Color(
                    c.red * BRIGHTNESS,
                    c.green * BRIGHTNESS,
                    c.blue * BRIGHTNESS));
        }
        led.setData(buffer);
    }

    @Override
    public void periodic() {
        boolean beambreak = m_IndexerSubsystem.getBeambreak();
        double intakeCurrent = m_IntakeSubsystem.getRollerSupplyCurrent();
        boolean indexerSpinning = m_IndexerSubsystem.getBallTunnelVelocity() > 1;
        var hopperState = m_IntakeSubsystem.getHopperState();

        boolean isIntaking = intakeCurrent > INTAKE_CURRENT_THRESHOLD;
        boolean isShooting = indexerSpinning;

        if (beambreak && !prevBeambreak) {
            beambreakBlinkStartMs = System.currentTimeMillis();
        }
        prevBeambreak = beambreak;

        boolean inBeambreakBlink = beambreakBlinkStartMs >= 0
                && (System.currentTimeMillis() - beambreakBlinkStartMs) < BEAMBREAK_BLINK_DURATION_MS;

        if (isShooting) {
            currentPattern = inBeambreakBlink ? getOffPattern() : getSolidGreenPattern();
        } else if (isIntaking) {
            currentPattern = getIntakingPattern();
        } else {
            currentPattern = getIdlePattern();
        }

        // Decide hopper fill count based on hopper state: EMPTY=10%, HALF=50%,
        // FULL=100%
        int leftStart = Leds.startIndexes[0];
        int leftEnd = Leds.startIndexes[1];
        int rightStart = Leds.startIndexes[1];
        int rightEnd = Leds.startIndexes[2];
        int leftLen = leftEnd - leftStart;
        int rightLen = rightEnd - rightStart;
        int totalHopper = leftLen + rightLen;

        double fillPercent;
        switch (hopperState) {
            case FULL:
                fillPercent = 1.0;
                break;
            case HALF:
                fillPercent = 0.5;
                break;
            case EMPTY:
            default:
                fillPercent = 0.10;
                break;
        }

        int fillCount = (int) Math.ceil(totalHopper * fillPercent);
        fillCount = Math.max(0, Math.min(fillCount, totalHopper));

        updateLeds(fillCount);
    }

    @Override
    public Sendable log() {
        return null;
    }
}