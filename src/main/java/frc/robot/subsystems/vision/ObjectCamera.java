package frc.robot.subsystems.vision;

import java.util.OptionalDouble;
import java.util.function.Supplier;

public class ObjectCamera {
    public Supplier<OptionalDouble> getObjectOffset() {
        return () -> OptionalDouble.empty();
    }
}
