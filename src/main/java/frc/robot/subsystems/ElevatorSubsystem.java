package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.Arm;
import frc.robot.subsystems.Elevator.Gripper;
import frc.robot.subsystems.Elevator.Winch;

public class ElevatorSubsystem extends SubsystemBase {

    private final static ElevatorSubsystem INSTANCE = new ElevatorSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static ElevatorSubsystem getInstance() {
        return INSTANCE;
    }

    private final Gripper gripper;
    private final Arm arm;
    private final Winch winch;

    private ElevatorSubsystem() {
        gripper = Gripper.getInstance();
        arm = Arm.getInstance();
        winch = Winch.getInstance();
    }

    @Override
    public void periodic() {}

    public void reset() {
//        elevator.setSetpoint(ElevatorConstants.kMinElevatorHeight);
//        arm.setSetpoint(ElevatorConstants.kMinArmHeight);
        gripper.setWristSetpoint(-Math.PI/2);
        arm.setSetpoint(Math.PI);
        winch.setSetpoint(0);
        SmartDashboard.putNumber("ArmAngleRads", arm.getEncoder());
        SmartDashboard.putNumber("WinchAngleRads", winch.getEncoder());
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

    public void setArmSpeed(double speed) {
        if(speed == 0) return;
        arm.setSetpoint(arm.getEncoder() + speed/ElevatorConstants.kPArm);
    }

    public void setWristSpeed(double speed) {
        if(speed == 0) return;
        gripper.setWristSetpoint(gripper.getWristEncoder() + speed/ElevatorConstants.kPWrist);
    }
    public void setWinchSpeed(double speed) {
        if(speed == 0) return;
        winch.setSetpoint(winch.getEncoder() + speed/ElevatorConstants.kPWinch);
    }

    public void freeControl(boolean isFreeControl) {
        arm.setFreeControl(isFreeControl);
        winch.setFreeControl(isFreeControl);
    }

    public void stop() {
        gripper.stop();
        winch.stop();
    }
}

