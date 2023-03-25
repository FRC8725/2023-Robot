package frc.robot.subsystems;


import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Gripper;

public class GripperSubsystem extends SubsystemBase {

    private final static GripperSubsystem INSTANCE = new GripperSubsystem();


    @SuppressWarnings("WeakerAccess")
    public static GripperSubsystem getInstance() {
        return INSTANCE;
    }

    Gripper gripper;

    NetworkTable led_nt = NetworkTableInstance.getDefault().getTable("LEDs");
    BooleanPublisher getItemPub = led_nt.getBooleanTopic("getItem").publish();

    // double lastUsedTime;
    // double startTime;

    // boolean firstLoop;
//    private boolean isResetting;

    private GripperSubsystem() {
        Timer.delay(3);
        gripper = Gripper.getInstance();
        // lastUsedTime = Timer.getFPGATimestamp();
//        reset();
        rangeTimer.reset();
        rangeTimer.start();
    }

    @Override
    public void periodic() {
        // if (Timer.getFPGATimestamp() - lastUsedTime > 4) {
        //     gripper.setDistanceEnable(false);
        //     firstLoop = true;
        // } 
        // else {
        //     gripper.setDistanceEnable(true);
        // }
        gripper.putSmartDashboard();
        gripper.autoSwitchDistance();
        if (rangeTimer.get() > 0.1) getItemPub.set(false);
//        gripper.calculate();
//        if (gripper.atRollSetpoint()) isResetting = false;
        // gripper.enableDistance();
        // SmartDashboard.putNumber("Distance Sensor", gripper.getDistanceSensor());
//        SmartDashboard.putBoolean("atRollSetpoint", atRollSetpoint());
    }

    Timer rangeTimer = new Timer();
    public boolean isPiecesInRange(boolean isFar) {
        // gripper.setDistanceEnable(true);  
        // lastUsedTime = Timer.getFPGATimestamp();
        // if (firstLoop) {
        //     startTime = Timer.getFPGATimestamp();
        //     firstLoop = false;
        // }
        // if (Timer.getFPGATimestamp() - startTime < 1) return false;
        var isInRange = gripper.getDistanceSensor() < 7.5 + (isFar? 0.5: 0);
        if (isInRange) {
            getItemPub.set(true);
            rangeTimer.reset();
        }
//        if (isInRange) led_nt.putValue("getItem", NetworkTableValue.makeBoolean(true, 2));
        return isInRange;
    }

    public boolean isDistanceOn() {
        return gripper.isDistanceRangeValid();
    }

    public void killDistance() {
        gripper.disableDistance();
    }

//    public void reset() {
//        gripper.setRollSetpoint(0);
//        isResetting = true;
//    }
//
//    public void setRollSetpoint(double setpoint) {
//        gripper.setRollSetpoint(setpoint);
//    }
//
//    public void addRollSetpoint(double variable) {
//        if (isResetting) return;
//        gripper.setRollSetpoint(gripper.getRollEncoder() + variable);
//    }
//
//    public boolean atRollSetpoint() {
//        return gripper.atRollSetpoint();
//    }


//    public void stop() {
//        gripper.setRollSetpoint(gripper.getRollEncoder());
//    }
}

