package frc.robot.Util;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Random;

import ca.team4308.absolutelib.leds.LEDConstants;

public class Patterns {
    public static final int DEFAULT_PATTERN_LENGTH = 30;
    public static final int DEFAULT_SCROLL_SPEED = 1;
    public static final double DEFAULT_BREATHE_PERIOD = 2.0;
    public static final double DEFAULT_BLINK_PERIOD = 0.2;

    public static LEDPattern idle() {
        return createBreathingPattern(LEDConstants.DEFAULT_IDLE_COLOR, LEDConstants.getBreathePeriod());
    }

    public static LEDPattern warning() {
        return createBlinkingPattern(LEDConstants.DEFAULT_WARNING_COLOR, LEDConstants.getBlinkPeriod());
    }

    public static LEDPattern error() {
        return createBlinkingPattern(LEDConstants.DEFAULT_ERROR_COLOR, LEDConstants.getBlinkPeriod() * 0.5);
    }

    public static LEDPattern success() {
        return createSolidPattern(LEDConstants.DEFAULT_SUCCESS_COLOR);
    }

    public static LEDPattern rainbowChase() {
        return createRainbowPattern(
                LEDConstants.DEFAULT_RAINBOW_SATURATION,
                LEDConstants.DEFAULT_RAINBOW_VALUE,
                LEDConstants.DEFAULT_RAINBOW_SPEED,
                1.0 / LEDConstants.getPatternLength());
    }

    public static LEDPattern getAlliancePattern(Color color) {
        return createSolidPattern(color);
    }

    public static LEDPattern getAllianceBreathing(Color color, double periodSeconds) {
        return createBreathingPattern(color, periodSeconds);
    }

    public static LEDPattern createRainbowPattern(int saturation, int value, double speedMPS, double spacing) {
        return new LEDPattern() {
            private final long startTime = System.currentTimeMillis();

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                // Offset shifts over time so the rainbow scrolls
                double elapsedSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
                int scrollOffset = (int) (elapsedSeconds * speedMPS * view.getLength()) % view.getLength();

                int length = view.getLength();
                for (int i = 0; i < length; i++) {
                    int hue = (int) (((i + scrollOffset) * 180.0 / length) % 180);
                    view.setHSV(i, hue, saturation, value);
                }
            }
        };
    }

    public static LEDPattern createBreathingPattern(Color baseColor, double periodSeconds) {
        return new LEDPattern() {
            private final long startTime = System.currentTimeMillis();

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                double phase = (time % periodSeconds) / periodSeconds;
                double intensity = (Math.sin(phase * 2 * Math.PI) + 1) / 2;

                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, new Color(
                            baseColor.red * intensity,
                            baseColor.green * intensity,
                            baseColor.blue * intensity));
                }
            }
        };
    }

    public static LEDPattern createBlinkingPattern(Color color, double periodSeconds) {
        return new LEDPattern() {
            private final long startTime = System.currentTimeMillis();

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                boolean isOn = ((time % periodSeconds) < (periodSeconds / 2));

                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, isOn ? color : Color.kBlack);
                }
            }
        };
    }

    public static LEDPattern createSolidPattern(Color color) {
        return new LEDPattern() {
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, color);
                }
            }
        };
    }

    public static LEDPattern createProgressPattern(Color color, double progress) {
        return new LEDPattern() {
            @Override
            public void applyTo(AddressableLEDBufferView view) {
                int activeLength = (int) (view.getLength() * progress);
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, i < activeLength ? color : Color.kBlack);
                }
            }
        };
    }

    public static LEDPattern chasingDot(Color color) {
        return new LEDPattern() {
            private final long startTime = System.currentTimeMillis();

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                double time = (System.currentTimeMillis() - startTime) / 1000.0;
                int position = (int) (time * LEDConstants.DEFAULT_CHASE_SPEED * view.getLength()) % view.getLength();

                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, i == position ? color : Color.kBlack);
                }
            }
        };
    }

    /**
     * Scrolling idle — time-based so applyTo is idempotent within a frame.
     * scrollSpeed is in pixels-per-second (was pixels-per-frame; multiply by ~50
     * if you want to keep the same visual speed as before).
     */
    public static LEDPattern scrollingIdle(Color baseColor, double pixelsPerSecond) {
        return new LEDPattern() {
            private final long startTime = System.currentTimeMillis();

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                int patternLength = LEDConstants.getPatternLength();
                int trailLength = patternLength / 4;
                int cycle = patternLength * 3;

                double elapsedSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
                int offset = (int) (elapsedSeconds * pixelsPerSecond) % cycle;
                if (offset < 0)
                    offset += cycle; // guard against sign issues

                for (int i = 0; i < view.getLength(); i++) {
                    int position = (i + offset) % cycle;
                    LEDUtils.setIdlePatternColor(view, i, position, baseColor, patternLength, trailLength);
                }
            }
        };
    }

    public static LEDPattern defaultScrollingIdle() {
        // 50 px/s ≈ the old scrollSpeed=1 at 50 Hz
        return scrollingIdle(getAllianceColor(), 50);
    }

    /**
     * ALT-F4 pattern — delta time is computed from wall clock, not frame count,
     * so multiple applyTo calls in the same frame have ~0 delta and are harmless.
     */
    public static LEDPattern altF4Pattern(double expandSpeed) {
        return new LEDPattern() {
            private final Random random = new Random();
            private final Color white = new Color(1, 1, 1);
            private final Color teamColor = getAllianceColor();
            private boolean isTeamColorPhase = true;
            private int[] expansionCenters = new int[0];
            private double[] expansionSizes = new double[0];
            private long lastUpdateTime = System.currentTimeMillis();
            private double timeSinceLastDot = 0;

            @Override
            public void applyTo(AddressableLEDBufferView view) {
                long currentTime = System.currentTimeMillis();
                double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
                lastUpdateTime = currentTime;

                // Skip near-zero deltas (same-frame duplicate calls)
                if (deltaTime < 0.001) {
                    renderCurrentState(view);
                    return;
                }

                timeSinceLastDot += deltaTime;

                Color baseColor = isTeamColorPhase ? white : teamColor;
                for (int i = 0; i < view.getLength(); i++) {
                    view.setColor(i, baseColor);
                }

                if (expansionCenters.length == 0 || isFullyExpanded(view.getLength())) {
                    if (timeSinceLastDot >= 1.0) {
                        addNewExpansionPoint(view.getLength());
                        timeSinceLastDot = 0;
                    }
                }

                updateExpansions(deltaTime, expandSpeed, view);
            }

            private void renderCurrentState(AddressableLEDBufferView view) {
                Color baseColor = isTeamColorPhase ? white : teamColor;
                for (int i = 0; i < view.getLength(); i++)
                    view.setColor(i, baseColor);
                updateExpansions(0, expandSpeed, view);
            }

            private void addNewExpansionPoint(int length) {
                if (expansionCenters.length == 0 || isFullyExpanded(length)) {
                    isTeamColorPhase = !isTeamColorPhase;
                    expansionCenters = new int[] { random.nextInt(length) };
                    expansionSizes = new double[] { 0 };
                }
            }

            private void updateExpansions(double deltaTime, double speed, AddressableLEDBufferView view) {
                Color dotColor = isTeamColorPhase ? teamColor : white;
                for (int i = 0; i < expansionSizes.length; i++) {
                    expansionSizes[i] += speed * deltaTime;
                    int radius = (int) expansionSizes[i];
                    int center = expansionCenters[i];
                    for (int j = center - radius; j <= center + radius; j++) {
                        int idx = wrapIndex(j, view.getLength());
                        double distance = Math.abs(j - center);
                        double fade = radius > 0 ? 1.0 - (distance / radius) : 1.0;
                        if (idx >= 0 && idx < view.getLength() && fade > 0) {
                            view.setColor(idx, blendColors(view.getLED(idx), dotColor, fade));
                        }
                    }
                }
            }

            private boolean isFullyExpanded(int length) {
                return expansionSizes.length == 0 || expansionSizes[0] >= length / 2.0;
            }

            private int wrapIndex(int index, int length) {
                if (index < 0)
                    return length + (index % length);
                return index % length;
            }

            private Color blendColors(Color c1, Color c2, double blend) {
                return new Color(
                        c1.red + (c2.red - c1.red) * blend,
                        c1.green + (c2.green - c1.green) * blend,
                        c1.blue + (c2.blue - c1.blue) * blend);
            }
        };
    }

    public static LEDPattern altF4() {
        return altF4Pattern(10.0);
    }

    private static Color getAllianceColor() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Blue ? new Color(0, 0, 1) : new Color(1, 0, 0);
    }
}