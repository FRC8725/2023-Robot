package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator.Arm;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.Gripper;
import frc.robot.subsystems.Elevator.Winch;

public class ElevatorSubsystem extends SubsystemBase {

    private final static ElevatorSubsystem INSTANCE = new ElevatorSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ElevatorSubsystem getInstance() {
        return INSTANCE;
    }

    private final Gripper gripper;
    private final Elevator elevator;
    private final Arm arm;
    private final Winch winch;

    private ElevatorSubsystem() {
        elevator = Elevator.getInstance();
        gripper = Gripper.getInstance();
        arm = Arm.getInstance();
        winch = Winch.getInstance();
    }

    @Override
    public void periodic() {
        if(elevator.getLimitSwitch()) {
            elevator.zeroEncoder();
            elevator.setSetpoint(ElevatorConstants.kMinElevatorHeight);
        }
        if(arm.getLimitSwitch()) {
            arm.zeroEncoder();
            arm.setSetpoint(ElevatorConstants.kMinArmHeight);
        }
    }

    public void reset() {
        elevator.setSetpoint(ElevatorConstants.kMinElevatorHeight);
        arm.setSetpoint(ElevatorConstants.kMinArmHeight);
        gripper.setWristSetpoint(-Math.PI);
        winch.setSetpoint(Math.PI);
        Timer.delay(1.5);
        elevator.set(-.08);
        arm.set(-.08);
    }

    public void setElevatorSetpoint(double setpoint) {
        elevator.setSetpoint(setpoint);
    }

    public void setArmSetpoint(double setpoint) {
        arm.setSetpoint(setpoint);
    }

    public void setWristSetpoint(double setpoint) {
        gripper.setWristSetpoint(setpoint);
    }

    public void setWinchSetpoint(double setpoint) {
        winch.setSetpoint(setpoint);
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
    public void setWinchSpeed(double speed) {
        winch.setSetpoint(winch.getSetpoint() + speed);
    }

    public void setGripperHorizontal() {
        if(DriverStation.isAutonomous()) return;
        gripper.setWristSetpoint(-winch.getSetpoint());
    }

    public void runIntake(boolean run, boolean isInverted) {
        double speed = ElevatorConstants.kIntakeSpeed * (isInverted? -1: 1);
        gripper.runIntake(run?speed: 0);
    }

    public boolean getIntakeSwitch() {
        return gripper.getIntakeLimitSwitch();
    }

    public void stop() {
        arm.set(0);
        elevator.set(0);
        gripper.stop();
        winch.stop();
    }
}

