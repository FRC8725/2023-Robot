package frc.robot.commands;

import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;


public class AutoLEDs extends CommandBase {

    LEDSubsystem ledSubsystem;

    NetworkTable led_nt = NetworkTableInstance.getDefault().getTable("LEDs");
    NetworkTable vision_nt = NetworkTableInstance.getDefault().getTable("Vision");

    BooleanSubscriber isGetItemSub = led_nt.getBooleanTopic("getItem").subscribe(false);
    BooleanSubscriber isLimelightSub = led_nt.getBooleanTopic("isLimelight").subscribe(false);
    IntegerSubscriber where2goSub = led_nt.getIntegerTopic("where2go").subscribe(0);
    IntegerSubscriber what2GrabSub = led_nt.getIntegerTopic("what2grab").subscribe(0);
    BooleanPublisher isLimelightPub = led_nt.getBooleanTopic("isLimelight").publish();
    BooleanPublisher isGetItemPub = led_nt.getBooleanTopic("getItem").publish();
    IntegerPublisher what2grabPub =  led_nt.getIntegerTopic("what2grab").publish();
    IntegerPublisher where2goPub =  led_nt.getIntegerTopic("where2go").publish();

    BooleanSubscriber isConeSub = vision_nt.getBooleanTopic("isCone").subscribe(false);

    double startTime;

    public AutoLEDs(LEDSubsystem ledSubsystem) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.ledSubsystem = ledSubsystem;
        startTime = Timer.getFPGATimestamp();
        addRequirements(ledSubsystem);
        ledSubsystem.setIdle(true);
    }

    @Override
    public void initialize() {
        isLimelightPub.set(false);
        isGetItemPub.set(false);
        what2grabPub.set(0);
        where2goPub.set(0);
    }

    boolean isIn = false;

    @Override
    public void execute() {
        ledSubsystem.setIdle(DriverStation.isDisabled());
        if (DriverStation.isAutonomousEnabled()) {
            ledSubsystem.autoMode();
            return;
        }

        // Read the NetworkTable Data
        boolean isGetItem = isGetItemSub.get();
        boolean isLimelight = isLimelightSub.get();
        boolean isCone = isConeSub.get();
        int where2go = (int) where2goSub.get();
        var what2Grab = what2GrabSub.get();

        // Set the LEDs of the arm
        if (isGetItem) {
            ledSubsystem.setBackColor(Color.kLimeGreen);
            ledSubsystem.setFrontColor(Color.kLimeGreen);
            if (!isIn) {
                startTime = Timer.getFPGATimestamp();
            } else if (Timer.getFPGATimestamp() - startTime > 1.5) {
                isGetItemPub.set(false);
            }
            isIn = true;
        } else if (isLimelight) {
            ledSubsystem.rainbow();
        } else {
            if (SmartDashboard.getBoolean("isGripperOpen", false)) isIn = false;
            // if (isIn) {
            //     if (isCone) {
            //         ledSubsystem.setFrontColor(Color.kYellow);
            //         ledSubsystem.setBackColor(Color.kYellow);
            //     }
            //     else {
            //         ledSubsystem.setFrontColor(Color.kPurple);
            //         ledSubsystem.setBackColor(Color.kPurple);
            //     }
            // } else {
            ledSubsystem.setBackColor(Color.fromHSV(0, 0, 0));
            switch (where2go) {
                case 0:
                    ledSubsystem.setFrontColor(Color.fromHSV(0, 0, 0), 0);
                    ledSubsystem.setFrontColor(what2Grab == 1? Color.kYellow: Color.kPurple, 1);
                    ledSubsystem.setBackColor(Color.fromHSV(0, 0, 0), 0);
                    ledSubsystem.setBackColor(what2Grab == 1? Color.kYellow: Color.kPurple, 1);     
                    break;
                case 1:
                    ledSubsystem.setFrontColor(Color.kCyan, 0);
                    ledSubsystem.setFrontColor(what2Grab == 1? Color.kYellow: Color.kPurple, 1);
                    ledSubsystem.setBackColor(Color.kCyan, 0);
                    ledSubsystem.setBackColor(what2Grab == 1? Color.kYellow: Color.kPurple, 1);
                    break;
                case 2:
                    ledSubsystem.setFrontColor(Color.kGhostWhite, 0);
                    ledSubsystem.setFrontColor(what2Grab == 1? Color.kYellow: Color.kPurple, 1);
                    ledSubsystem.setBackColor(Color.kGhostWhite, 0);
                    ledSubsystem.setBackColor(what2Grab == 1? Color.kYellow: Color.kPurple, 1);
                    break;
                // }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.setIdle(true);
    }
}
