package frc.robot.Util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Razer2Wrapper {
    public static class RazerMapping {
        public static int A = 1;
        public static int B = 2;
        public static int X = 3;
        public static int Y = 4;

        public static int LB = 5;
        public static int RB = 6;

        public static int Start = 7;
        public static int Back = 8;

        public static int LeftStickButton = 9;
        public static int RightStickButton = 10;
    }

    public final Joystick joystick1;
    public final Joystick joystick2;

    public final Trigger A;
    public final Trigger B;
    public final Trigger X;
    public final Trigger Y;

    public final Trigger LB;
    public final Trigger RB;

    public final Trigger M1;
    public final Trigger M2;
    public final Trigger M3;
    public final Trigger M4;
    public final Trigger M5;
    public final Trigger M6;

    public final Trigger LeftStickButton;
    public final Trigger RightStickButton;

    public final Trigger RightTrigger;
    public final Trigger LeftTrigger;

    public final Trigger Start;
    public final Trigger Back;

    public final Trigger povUp;
    public final Trigger povRight;
    public final Trigger povDown;
    public final Trigger povLeft;

    public Razer2Wrapper(int port1, int port2) {
        this.joystick1 = new Joystick(port1);
        this.joystick2 = new Joystick(port2);

        this.A = new JoystickButton(joystick1, RazerMapping.A)
                .or(new JoystickButton(joystick2, RazerMapping.A));
        this.B = new JoystickButton(joystick1, RazerMapping.B)
                .or(new JoystickButton(joystick2, RazerMapping.B));
        this.X = new JoystickButton(joystick1, RazerMapping.X)
                .or(new JoystickButton(joystick2, RazerMapping.X));
        this.Y = new JoystickButton(joystick1, RazerMapping.Y)
                .or(new JoystickButton(joystick2, RazerMapping.Y));

        this.LB = new JoystickButton(joystick1, RazerMapping.LB)
                .or(new JoystickButton(joystick2, RazerMapping.LB));
        this.RB = new JoystickButton(joystick1, RazerMapping.RB)
                .or(new JoystickButton(joystick2, RazerMapping.RB));

        this.LeftStickButton = new JoystickButton(joystick1, RazerMapping.LeftStickButton)
                .or(new JoystickButton(joystick2, RazerMapping.LeftStickButton));
        this.RightStickButton = new JoystickButton(joystick1, RazerMapping.RightStickButton)
                .or(new JoystickButton(joystick2, RazerMapping.RightStickButton));

        this.M1 = new JoystickButton(joystick1, RazerMapping.Start)
                .or(new JoystickButton(joystick2, RazerMapping.Start));
        this.M2 = new JoystickButton(joystick1, RazerMapping.Back)
                .or(new JoystickButton(joystick2, RazerMapping.Back));

        this.M3 = new Trigger(() -> joystick1.getPOV() == 0 || joystick2.getPOV() == 0);
        this.M4 = new Trigger(() -> joystick1.getPOV() == 180 || joystick2.getPOV() == 180);
        this.M5 = new Trigger(() -> joystick1.getPOV() == 270 || joystick2.getPOV() == 270);
        this.M6 = new Trigger(() -> joystick1.getPOV() == 90 || joystick2.getPOV() == 90);

        this.RightTrigger = new Trigger(() -> getRightTriggerAsBoolean());
        this.LeftTrigger = new Trigger(() -> getLeftTriggerAsBoolean());

        this.Start = new JoystickButton(joystick1, RazerMapping.Start)
                .or(new JoystickButton(joystick2, RazerMapping.Start));
        this.Back = new JoystickButton(joystick1, RazerMapping.Back)
                .or(new JoystickButton(joystick2, RazerMapping.Back));

        this.povUp = new POVButton(joystick1, 0)
                .or(new POVButton(joystick2, 0));
        this.povRight = new POVButton(joystick1, 90)
                .or(new POVButton(joystick2, 90));
        this.povDown = new POVButton(joystick1, 180)
                .or(new POVButton(joystick2, 180));
        this.povLeft = new POVButton(joystick1, 270)
                .or(new POVButton(joystick2, 270));
    }

    private double sumAxes(double a, double b) {
        return Math.max(-1.0, Math.min(1.0, a + b));
    }

    public double getLeftX() {
        return sumAxes(joystick1.getRawAxis(0), joystick2.getRawAxis(0));
    }

    public double getLeftY() {
        return sumAxes(joystick1.getRawAxis(1), joystick2.getRawAxis(1));
    }

    public double getRightX() {
        return sumAxes(joystick1.getRawAxis(4), joystick2.getRawAxis(4));
    }

    public double getRightY() {
        return sumAxes(joystick1.getRawAxis(5), joystick2.getRawAxis(5));
    }

    public double getLeftTrigger() {
        return sumAxes(joystick1.getRawAxis(2), joystick2.getRawAxis(2));
    }

    public double getRightTrigger() {
        return sumAxes(joystick1.getRawAxis(3), joystick2.getRawAxis(3));
    }

    public boolean getLeftTriggerAsBoolean() {
        return joystick1.getRawAxis(2) > 0.5 || joystick2.getRawAxis(2) > 0.5;
    }

    public boolean getRightTriggerAsBoolean() {
        return joystick1.getRawAxis(3) > 0.5 || joystick2.getRawAxis(3) > 0.5;
    }

    public boolean getLeftTriggerAsBoolean(double threshold) {
        return joystick1.getRawAxis(2) > threshold || joystick2.getRawAxis(2) > threshold;
    }

    public boolean getRightTriggerAsBoolean(double threshold) {
        return joystick1.getRawAxis(3) > threshold || joystick2.getRawAxis(3) > threshold;
    }

    public void setRumble(RumbleType type, double power) {
        joystick1.setRumble(type, power);
        joystick2.setRumble(type, power);
    }
}