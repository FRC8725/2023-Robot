package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;


public class RunElevatorToPosition extends CommandBase {

    ElevatorSubsystem elevatorSubsystem;
    double elevatorPosition;
    double armPosition;
    double winchAngle;
    double wristAngle;

    public RunElevatorToPosition(ElevatorSubsystem elevatorSubsystem,
                                 double elevatorPosition,
                                 double armPosition,
                                 double winchAngle,
                                 double wristAngle) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorPosition = elevatorPosition;
        this.armPosition = armPosition;
        this.winchAngle = winchAngle;
        this.wristAngle = wristAngle;
        addRequirements(elevatorSubsystem);
    }
    public RunElevatorToPosition(ElevatorSubsystem elevatorSubsystem,
                                 double[] elevatorPose) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorPosition = elevatorPose[0];
        this.armPosition = elevatorPose[1];
        this.winchAngle = elevatorPose[2];
        this.wristAngle = elevatorPose[3];
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
//        elevatorSubsystem.setElevatorSetpoint(elevatorPosition);
//        elevatorSubsystem.setArmSetpoint(armPosition);
//        elevatorSubsystem.setWinchSetpoint(winchAngle);
//        elevatorSubsystem.setWristSetpoint(wristAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
