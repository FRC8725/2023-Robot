package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.Gripper;

public class GripperSubsystem extends SubsystemBase {

    private final static GripperSubsystem INSTANCE = new GripperSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static GripperSubsystem getInstance() {
        return INSTANCE;
    }

    Gripper gripper;

    private GripperSubsystem() {
        gripper = Gripper.getInstance();
    }

    public void runIntake(boolean run, boolean isInverted) {
        double speed = ElevatorConstants.kIntakeSpeed * (isInverted? -1: 1);
        gripper.runIntake(run?speed: 0);
    }

    public boolean getIntakeSwitch() {
        return gripper.getIntakeLimitSwitch();
    }

}

