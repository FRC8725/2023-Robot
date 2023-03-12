package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxJoystick extends CommandXboxController {
    public final Trigger btn_X = this.x();
    public final Trigger btn_A = this.a();
    public final Trigger btn_B = this.b();
    public final Trigger btn_Y = this.y();
    public final Trigger btn_topL = this.leftBumper();
    public final Trigger btn_topR = this.rightBumper();
    public final Trigger btn_triggerL = this.leftTrigger();
    public final Trigger btn_triggerR = this.rightTrigger();
    // setting POVButton
    public final Trigger POV_North = this.povUp();
    public final Trigger POV_NorthEast = this.povUpRight();
    public final Trigger POV_East = this.povRight();
    public final Trigger POV_SouthEast = this.povDownRight();
    public final Trigger POV_South = this.povDown();
    public final Trigger POV_SouthWest = this.povDownLeft();
    public final Trigger POV_West = this.povLeft();
    public final Trigger POV_NorthWest = this.povUpLeft();
    public final Trigger POV_Center = this.povCenter();

    public final Trigger btn_LStick = this.leftStick();
    public final Trigger btn_RStick = this.rightStick();
    public XboxJoystick(int port) {
        super(port);
    }

    // Setting Analog Stick
    public double get_LStickX() {
        return this.getLeftX();
    }

    public double get_LStickY() {
        return this.getLeftY();
    }

    public double get_RStickX() {
        return this.getRightX();
    }

    public double get_RStickY() {
        return this.getRightY();
    }

    public double get_LeftTriggerAxis() {
        return this.getLeftTriggerAxis();
    }

    public double get_RightTriggerAxis() {
        return this.getRightTriggerAxis();
    }
}