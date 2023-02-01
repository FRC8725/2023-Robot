package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
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
            elevator.setSpeed(0);
            elevator.setSetpoint(ElevatorConstants.kMinElevatorHeight);
        }
        if(arm.getLimitSwitch()) {
            arm.zeroEncoder();
            arm.set(0);
            arm.setSetpoint(ElevatorConstants.kMinArmHeight);
        }
    }

    public void reset() {
//        elevator.setSetpoint(ElevatorConstants.kMinElevatorHeight);
//        arm.setSetpoint(ElevatorConstants.kMinArmHeight);
        gripper.setWristSetpoint(-Math.PI/2);
        winch.setSetpoint(Math.PI/2);
        elevator.setSpeed(-.08);
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
        if(speed == 0) return;
        elevator.setSetpoint(elevator.getEncoder() + speed/ElevatorConstants.kPElevator*ElevatorConstants.kElevatorSpeed);
    }

    public void setArmSpeed(double speed) {
        if(speed == 0) return;
        arm.setSetpoint(arm.getEncoder() + speed/ElevatorConstants.kPArm*ElevatorConstants.kArmSpeed);
    }

    public void setWristSpeed(double speed) {
        if(speed == 0) return;
        gripper.setWristSetpoint(gripper.getWristEncoder() + speed/ElevatorConstants.kPWrist);
    }
    public void setWinchSpeed(double speed) {
        if(speed == 0) return;
        winch.setSetpoint(winch.getEncoder() + speed/ElevatorConstants.kPWinch);
    }

    public void setGripperHorizontal() {
        if(DriverStation.isAutonomous()) return;
        gripper.setWristSetpoint(-winch.getSetpoint());
    }

    public void stop() {
        arm.set(0);
        elevator.setSpeed(0);
        gripper.stop();
        winch.stop();
    }
}

