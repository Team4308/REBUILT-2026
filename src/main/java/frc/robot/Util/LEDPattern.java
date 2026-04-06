package frc.robot.Util;

public interface LEDPattern {
    /**
     * Applies the pattern to a buffer view.
     * 
     * @param view The buffer view to apply the pattern to
     */
    void applyTo(AddressableLEDBufferView view);
}
