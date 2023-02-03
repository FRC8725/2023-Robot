package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.Elbow;
import frc.robot.subsystems.Elevator.Gripper;
import frc.robot.subsystems.Elevator.Winch;

public class GripperSubsystem extends SubsystemBase {

    private final static GripperSubsystem INSTANCE = new GripperSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static GripperSubsystem getInstance() {
        return INSTANCE;
    }

    Gripper gripper;
    Elbow elbow;
    Winch winch;
    boolean isHorizontal;

    private GripperSubsystem() {
        gripper = Gripper.getInstance();
        elbow = Elbow.getInstance();
        winch = Winch.getInstance();
        reset();
        Timer.delay(2);
    }

    @Override
    public void periodic(){
        gripper.setWristSetpoint(elbow.getAbsoluteEncoderRad() - Math.PI/2 + winch.getAbsoluteEncoderRad() + (isHorizontal? 0: -90));
        SmartDashboard.putNumber("Wrist Setpoint", elbow.getEncoder() - Math.PI/2 + winch.getEncoder() + (isHorizontal? 0: -90));
    }

    public void reset() {
        gripper.resetWristEncoder();
        gripper.setRollSetpoint(0);
        isHorizontal = true;
    }

    public void setHorizontal(boolean isHorizontal) {
        this.isHorizontal = isHorizontal;
    }

    public void setRollSetpoint(double setpoint) {
        gripper.setRollSetpoint(setpoint);
    }

    public void addRollSetpoint(double variable) {
        gripper.setRollSetpoint(gripper.getRollEncoder()+variable);
    }

}

