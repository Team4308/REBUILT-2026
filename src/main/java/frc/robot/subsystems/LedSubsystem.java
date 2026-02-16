package frc.robot.subsystems;

import com.google.flatbuffers.Constants;

import ca.team4308.absolutelib.leds.AddressableLEDBufferView;
import ca.team4308.absolutelib.leds.LEDPattern;
import ca.team4308.absolutelib.leds.Patterns;
import ca.team4308.absolutelib.wrapper.AbsoluteSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.Leds;
import frc.robot.StateManager;

public class LedSubsystem extends AbsoluteSubsystem {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView view;

    private LEDPattern currentPattern;
    private String currentPatternName = "Idle";

    private static final int LED_PORT = Leds.LED_PORT;
    private static final int LED_LENGTH = Leds.LED_LENGTH;

    public LedSubsystem() {
        super();
        led = new AddressableLED(LED_PORT);
        buffer = new AddressableLEDBuffer(LED_LENGTH);
        led.setLength(LED_LENGTH);
        led.setData(buffer);
        led.start();

        view = new AddressableLEDBufferView(buffer, 0, LED_LENGTH);

        currentPattern = Patterns.idle();
    }

    @Override
    public void periodic() {
        if (currentPattern != null) {
            currentPattern.applyTo(view);
        }
        led.setData(buffer);
    }

    public void setIdle() {
        currentPattern = Patterns.defaultScrollingIdle();
        currentPatternName = "Idle";
    }

    public void setError() {
        currentPattern = Patterns.error();
        currentPatternName = "Error";
    }

    public void setSuccess() {
        currentPattern = Patterns.success();
        currentPatternName = "Success";
    }

    public void setWarning() {
        currentPattern = Patterns.warning();
        currentPatternName = "Warning";
    }

    public void setRainbow() {
        currentPattern = Patterns.rainbowChase();
        currentPatternName = "Rainbow";
    }

    public void setAlliance() {
        currentPattern = Patterns.getAlliancePattern();
        currentPatternName = "Alliance";
    }

    public void setAllianceBreathing() {
        currentPattern = Patterns.getAllianceBreathing(2.0);
        currentPatternName = "Alliance Breathing";
    }

    public void setSolidColor(Color color) {
        currentPattern = Patterns.createSolidPattern(color);
        currentPatternName = "Solid";
    }

    public void setProgress(double progress, Color color) {
        currentPattern = Patterns.createProgressPattern(color, progress);
        currentPatternName = "Progress";
    }

    public void setChasingDot(Color color) {
        currentPattern = Patterns.chasingDot(color);
        currentPatternName = "Chasing Dot";
    }

    public void applyStatePattern(StateManager.RobotState state) {
        String name = state.name();
        if (name.equals(currentPatternName)) {
            return;
        }

        switch (state) {
            case Home:
                currentPattern = Patterns.getAllianceBreathing(3.0);
                break;

            case ActiveTeleopAllianceZone:
                currentPattern = Patterns.createRainbowPattern(LED_LENGTH, 4, 1.0, 1.0);
                break;

            case ActiveTeleopNeutralZone:
                currentPattern = Patterns.chasingDot(Color.kGreen);
                break;

            case ActiveTeleopOpponentZone:
                currentPattern = Patterns.createBlinkingPattern(Color.kOrangeRed, 0.15);
                break;

            case InactiveTeleopAllianceZone:
                currentPattern = Patterns.scrollingIdle(Color.kCyan, 2);
                break;

            case InactiveTeleopNeutralZone:
                currentPattern = Patterns.createBreathingPattern(Color.kYellow, 1.5);
                break;

            case InactiveTeleopOpponentZone:
                currentPattern = Patterns.chasingDot(Color.kMagenta);
                break;

            case EndgameTeleopAllianceZone:
                    currentPattern = Patterns.createBlinkingPattern(Color.kRed, DriverStation.getMatchTime() / 30.0);
                    break;
            case EndgameTeleopNeutralZone:
                        currentPattern = Patterns.createBlinkingPattern(Color.kRed, DriverStation.getMatchTime() / 30.0);
                    break;
            case EndgameTeleopOpponentZone:
                currentPattern = Patterns.altF4();
                break;

            default:
                currentPattern = Patterns.defaultScrollingIdle();
                break;
        }

        currentPatternName = name;
    }

    public String getCurrentPatternName() {
        return currentPatternName;
    }

    @Override
    public Sendable log() {
        return builder -> {
            builder.setSmartDashboardType("LEDs");
            builder.addStringProperty("CurrentPattern", () -> currentPatternName, null);
            builder.addIntegerProperty("LEDCount", () -> LED_LENGTH, null);
        };
    }
}
