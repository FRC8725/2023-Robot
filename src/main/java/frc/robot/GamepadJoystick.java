package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class GamepadJoystick extends Joystick {
    public Trigger btn_X = new JoystickButton(this, 1);
    public Trigger btn_A = new JoystickButton(this, 2);
    public Trigger btn_B = new JoystickButton(this, 3);
    public Trigger btn_Y = new JoystickButton(this, 4);
    public Trigger btn_topL = new JoystickButton(this, 5);
    public Trigger btn_topR = new JoystickButton(this, 6);
    public Trigger btn_triggerL = new JoystickButton(this, 7);
    public Trigger btn_triggerR = new JoystickButton(this, 8);
    public Trigger btn_Start = new JoystickButton(this, 9);
    public Trigger btn_Back = new JoystickButton(this, 10);
    // setting POVButton
    public POVButton POV_North = new POVButton(this, 0);
    public POVButton POV_NorthEast = new POVButton(this, 45);
    public POVButton POV_East = new POVButton(this, 90);
    public POVButton POV_SouthEast = new POVButton(this, 135);
    public POVButton POV_South = new POVButton(this, 180);
    public POVButton POV_SouthWest = new POVButton(this, 225);
    public POVButton POV_West = new POVButton(this, 270);
    public POVButton POV_NorthWest = new POVButton(this, 315);
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