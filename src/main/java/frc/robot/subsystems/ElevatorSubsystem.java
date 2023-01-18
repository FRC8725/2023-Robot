package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Gripper;

public class ElevatorSubsystem extends SubsystemBase {

    private final static ElevatorSubsystem INSTANCE = new ElevatorSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ElevatorSubsystem getInstance() {
        return INSTANCE;
    }

    private final Gripper gripper;
    private final Elevator elevator;
    private final Arm arm;

    private ElevatorSubsystem() {
        elevator = Elevator.getInstance();
        gripper = Gripper.getInstance();
        arm = Arm.getInstance();
    }

    @Override
    public void periodic() {
        if(elevator.getLimitSwitch()) {
            elevator.zeroEncoder();
            elevator.setSetpoint(.15);
        }
        if(arm.getLimitSwitch()) {
            arm.zeroEncoder();
            arm.setSetpoint(.15);
        }
    }

    public void reset() {
        elevator.setSetpoint(.15);
        arm.setSetpoint(.15);
        gripper.setWristSetpoint(0);
        Timer.delay(1.5);
        elevator.set(-.1);
        arm.set(-.1);
    }

    public void setElevatorSetpoint(double setpoint) {
        elevator.setSetpoint(setpoint);
    }

    public void setElevatorSpeed(double speed) {
        elevator.setSetpoint(elevator.getSetpoint() + speed);
    }

    public void setArmSpeed(double speed) {
        arm.setSetpoint(arm.getSetpoint() + speed);
    }

    public void setWristSpeed(double speed) {
        gripper.setWristSetpoint(gripper.getWristSetpoint() + speed);
    }

    public void runIntake(boolean run) {
        gripper.runIntake(run?ElevatorConstants.kIntakeSpeed: 0);
    }

    public void stop() {
        arm.set(0);
        elevator.set(0);
        gripper.stop();
    }
}

