package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Gripper;

public class GripperSubsystem extends SubsystemBase {

    private final static GripperSubsystem INSTANCE = new GripperSubsystem();


    @SuppressWarnings("WeakerAccess")
    public static GripperSubsystem getInstance() {
        return INSTANCE;
    }

    Gripper gripper;

    private GripperSubsystem() {
        Timer.delay(2);
        gripper = Gripper.getInstance();
        reset();
    }

    @Override
    public void periodic() {
        gripper.calculate();
    }

    public void reset() {
        gripper.setRollSetpoint(0);
    }

    public void setRollSetpoint(double setpoint) {
        gripper.setRollSetpoint(setpoint);
    }

    public void addRollSetpoint(double variable) {
        gripper.setRollSetpoint(gripper.getRollEncoder() + variable);
    }
}

