package frc.robot.commands;

import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;


public class AutoLEDs extends CommandBase {

    LEDSubsystem ledSubsystem;

    NetworkTable led_nt = NetworkTableInstance.getDefault().getTable("LEDs");
    NetworkTable vision_nt = NetworkTableInstance.getDefault().getTable("Vision");

    public AutoLEDs(LEDSubsystem ledSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        led_nt.getBooleanTopic("isLimelight").publish().set(false);
        led_nt.getBooleanTopic("getItem").publish().set(false);
        led_nt.getIntegerTopic("what2grab").publish().set(0);
        led_nt.getIntegerTopic("where2go").publish().set(0);
    }

    boolean isIn = false;

    @Override
    public void execute() {

        if (DriverStation.isDisabled()) {
            ledSubsystem.setFrontColor(Color.kOrangeRed, 0);
            ledSubsystem.setFrontColor(Color.kOrangeRed, 1);
            ledSubsystem.setBackColor(Color.kOrangeRed);
        }

        if (led_nt.getBooleanTopic("getItem").subscribe(false).get()) {
            ledSubsystem.setBackColor(Color.kLimeGreen);
            if (!isIn) led_nt.getBooleanTopic("getItem").publish().set(false, NetworkTablesJNI.now() + 2500);
            isIn = true;
        } else if (led_nt.getBooleanTopic("isLimelight").subscribe(false).get()) {
            ledSubsystem.rainbow();
        } else {
            if (SmartDashboard.getBoolean("isGripperOpen", false)) isIn = false;
            if (isIn) {
                if (vision_nt.getBooleanTopic("isCone").subscribe(false).get()) ledSubsystem.setBackColor(Color.kYellow);
                else ledSubsystem.setBackColor(Color.kPurple);
            } else {
                ledSubsystem.setBackColor(Color.fromHSV(0, 0, 0));
            }
        }

        // 0 is Cube
        // 1 is Cone
        var what2Grab = led_nt.getIntegerTopic("what2grab").subscribe(0).get();

        if (isIn) {
            if (vision_nt.getBooleanTopic("isCone").subscribe(false).get()) {
                ledSubsystem.setFrontColor(Color.kYellow, 0);
                ledSubsystem.setFrontColor(Color.kYellow, 1);
            }
            else {
                ledSubsystem.setFrontColor(Color.kPurple, 0);
                ledSubsystem.setFrontColor(Color.kPurple, 1);
            }
        } else {
            switch ((int) led_nt.getIntegerTopic("where2go").subscribe(0).get()) {
                case 0:
                    ledSubsystem.setFrontColor(Color.kBrown, 0);
                    ledSubsystem.setFrontColor(Color.kBrown, 1);
                    break;
                case 1:
                    ledSubsystem.setFrontColor(Color.kLightBlue, 0);
                    ledSubsystem.setFrontColor(what2Grab == 1? Color.kYellow: Color.kPurple, 1);
                    break;
                case 2:
                    ledSubsystem.setFrontColor(Color.kWhite, 0);
                    ledSubsystem.setFrontColor(what2Grab == 1? Color.kYellow: Color.kPurple, 1);
                    break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.setFrontColor(Color.kOrangeRed, 0);
        ledSubsystem.setFrontColor(Color.kOrangeRed, 1);
        ledSubsystem.setBackColor(Color.kOrangeRed);
    }
}
