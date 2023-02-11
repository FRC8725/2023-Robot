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
    boolean isHorizontal;

    private GripperSubsystem() {
        gripper = Gripper.getInstance();
        reset();
    }

    public void reset() {
        gripper.setRollSetpoint(0);
        isHorizontal = true;
    }

    public void setRollSetpoint(double setpoint) {
        gripper.setRollSetpoint(setpoint);
    }

    public void addRollSetpoint(double variable) {
        gripper.setRollSetpoint(gripper.getRollEncoder()+variable);
    }
}

