package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamepadJoystick extends Joystick {
    public final Trigger btn_X = new JoystickButton(this, 1);
    public final Trigger btn_A = new JoystickButton(this, 2);
    public final Trigger btn_B = new JoystickButton(this, 3);
    public final Trigger btn_Y = new JoystickButton(this, 4);
    public final Trigger btn_topL = new JoystickButton(this, 5);
    public final Trigger btn_topR = new JoystickButton(this, 6);
    public final Trigger btn_triggerL = new JoystickButton(this, 7);
    public final Trigger btn_triggerR = new JoystickButton(this, 8);
    public final Trigger btn_Back = new JoystickButton(this, 9);
    public final Trigger btn_Start = new JoystickButton(this, 10);
    // setting POVButton
    public final POVButton POV_North = new POVButton(this, 0);
    public final POVButton POV_NorthEast = new POVButton(this, 45);
    public final POVButton POV_East = new POVButton(this, 90);
    public final POVButton POV_SouthEast = new POVButton(this, 135);
    public final POVButton POV_South = new POVButton(this, 180);
    public final POVButton POV_SouthWest = new POVButton(this, 225);
    public final POVButton POV_West = new POVButton(this, 270);
    public final POVButton POV_NorthWest = new POVButton(this, 315);
    public GamepadJoystick(int port) {
        super(port);
    }

    // Setting Analog Stick
    public double get_LStickX() {
        return getRawAxis(0);
    }

    public double get_LStickY() {
        return -getRawAxis(1);
    }

    public double get_RStickX() {
        return getRawAxis(2);
    }

    public double get_RStickY() {
        return -getRawAxis(3);
    }
}