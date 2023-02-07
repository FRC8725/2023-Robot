package frc.robot.subsystems;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Elbow;
import frc.robot.subsystems.Arm.Gripper;
import frc.robot.subsystems.Arm.Winch;

public class GripperSubsystem extends SubsystemBase {

    private final static GripperSubsystem INSTANCE = new GripperSubsystem();


    @SuppressWarnings("WeakerAccess")
    public static GripperSubsystem getInstance() {
        return INSTANCE;
    }

    Gripper gripper;
    Elbow elbow;
    Winch winch;
    Pneumatics pneumatics;
    boolean isHorizontal;

    private GripperSubsystem() {
        gripper = Gripper.getInstance();
        elbow = Elbow.getInstance();
        winch = Winch.getInstance();
        pneumatics = Pneumatics.getInstance();
        reset();
    }

    @Override
    public void periodic(){
        gripper.setWristSetpoint(elbow.getAbsoluteEncoderRad() - Math.PI/2 + winch.getAbsoluteEncoderRad() + (isHorizontal? 0: -Math.PI/2) + (pneumatics.getGripperStatus()? 0: Units.degreesToRadians(10)));
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

