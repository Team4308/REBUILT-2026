package frc.robot.Util;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Represents a view into a portion of an AddressableLEDBuffer.
 * Allows treating a section of LEDs as its own buffer.
 */
public class AddressableLEDBufferView {
    private final AddressableLEDBuffer buffer;
    private final int startIndex;
    private final int length;
    private final boolean reversed;

    /**
     * Creates a new view into an LED buffer.
     * 
     * @param buffer     The parent LED buffer
     * @param startIndex The starting index in the parent buffer
     * @param length     The number of LEDs in this view
     */
    public AddressableLEDBufferView(AddressableLEDBuffer buffer, int startIndex, int length, boolean reversed) {
        if (startIndex < 0 || length < 0 || startIndex + length > buffer.getLength()) {
            throw new IllegalArgumentException(
                    String.format("Invalid buffer view range: startIndex=%d, length=%d, bufferLength=%d",
                            startIndex, length, buffer.getLength()));
        }
        this.buffer = buffer;
        this.startIndex = startIndex;
        this.length = length;
        this.reversed = reversed;
    }

    public AddressableLEDBufferView(AddressableLEDBuffer buffer, int startIndex, int length) {
        this(buffer, startIndex, length, false);
    }

    private int resolve(int index) {
        return startIndex + (reversed ? (length - 1 - index) : index);
    }

    public void setColor(int index, Color color) {
        if (isValidIndex(index))
            buffer.setLED(resolve(index), color);
    }

    public void setRGB(int index, int r, int g, int b) {
        if (isValidIndex(index))
            buffer.setRGB(resolve(index), r, g, b);
    }

    public void setHSV(int index, int h, int s, int v) {
        if (isValidIndex(index))
            buffer.setHSV(resolve(index), h, s, v);
    }

    public Color getLED(int index) {
        if (isValidIndex(index))
            return buffer.getLED(resolve(index));
        return new Color(0, 0, 0);
    }

    /**
     * Gets the length of this buffer view.
     * 
     * @return The number of LEDs in this view
     */
    public int getLength() {
        return length;
    }

    /**
     * Gets the starting index of this view in the parent buffer.
     * 
     * @return The start index
     */
    public int getStartIndex() {
        return startIndex;
    }

    private boolean isValidIndex(int index) {
        return index >= 0 && index < length;
    }
}
