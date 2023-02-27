package frc.robot.subsystems;


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
//    private boolean isResetting;

    private GripperSubsystem() {
        Timer.delay(2);
        gripper = Gripper.getInstance();
//        reset();
    }

    @Override
    public void periodic() {
//        gripper.calculate();
//        if (gripper.atRollSetpoint()) isResetting = false;
        SmartDashboard.putNumber("Distance Sensor", gripper.getDistanceSensor());
//        SmartDashboard.putBoolean("atRollSetpoint", atRollSetpoint());
    }

    public boolean isPiecesInRange() {
        return gripper.getDistanceSensor() < 180;
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

