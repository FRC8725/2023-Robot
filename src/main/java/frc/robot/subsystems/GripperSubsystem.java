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

    double lastUsedTime;
    double startTime;

    boolean firstLoop;
//    private boolean isResetting;

    private GripperSubsystem() {
        Timer.delay(2);
        gripper = Gripper.getInstance();
        lastUsedTime = Timer.getFPGATimestamp();
//        reset();
    }

    @Override
    public void periodic() {
        if (Timer.getFPGATimestamp() - lastUsedTime > 3) {
            gripper.setDistanceEnable(false);
            firstLoop = true;
        } 
        else {
            gripper.setDistanceEnable(true);
        }
//        gripper.calculate();
//        if (gripper.atRollSetpoint()) isResetting = false;
        // gripper.enableDistance();
        // SmartDashboard.putNumber("Distance Sensor", gripper.getDistanceSensor());
//        SmartDashboard.putBoolean("atRollSetpoint", atRollSetpoint());
    }

    public boolean isPiecesInRange(boolean isFar) {
        // gripper.setDistanceEnable(true);  
        lastUsedTime = Timer.getFPGATimestamp();
        if (firstLoop) {
            startTime = Timer.getFPGATimestamp();
            firstLoop = false;
        }
        if (Timer.getFPGATimestamp() - startTime < 2) return false;
        var isInRange = gripper.getDistanceSensor() < 200 + (isFar? 50: 0);
        if (isInRange) getItemPub.set(true);
//        if (isInRange) led_nt.putValue("getItem", NetworkTableValue.makeBoolean(true, 2));
        return isInRange;
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

