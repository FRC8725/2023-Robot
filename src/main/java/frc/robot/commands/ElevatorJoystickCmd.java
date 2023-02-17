package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.Supplier;


public class ElevatorJoystickCmd extends CommandBase {

    ElevatorSubsystem elevatorSubsystem;
    Supplier<Double> elevatorSpdFunction;
    Supplier<Double> armSpdFunction;
    Supplier<Double> wristSpdFunction;
    Supplier<Double> winchSpdFunction;
    Supplier<Boolean> gripperHorizontalFunction;

    public ElevatorJoystickCmd(ElevatorSubsystem elevatorSubsystem,
                               Supplier<Double> elevatorSpdFunction,
                               Supplier<Double> armSpdFunction,
                               Supplier<Double> wristSpdFunction,
                               Supplier<Double> winchSpdFunction,
                               Supplier<Boolean> gripperHorizontalFunction) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorSpdFunction = elevatorSpdFunction;
        this.armSpdFunction = armSpdFunction;
        this.wristSpdFunction = wristSpdFunction;
        this.winchSpdFunction = winchSpdFunction;
        this.gripperHorizontalFunction = gripperHorizontalFunction;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var elevatorSpeed = elevatorSpdFunction.get();
        elevatorSpeed = Math.abs(elevatorSpeed) > Constants.Joystick.DEADBAND ? elevatorSpeed: 0.0;
        elevatorSubsystem.setElevatorSpeed(elevatorSpeed);
        var armSpeed = armSpdFunction.get();
        armSpeed = Math.abs(armSpeed) > Constants.Joystick.DEADBAND ? armSpeed: 0;
        elevatorSubsystem.setArmSpeed(armSpeed);
        var wristSpeed = wristSpdFunction.get();
        wristSpeed = Math.abs(wristSpeed) > Constants.Joystick.DEADBAND ? wristSpeed: 0;
        elevatorSubsystem.setWristSpeed(wristSpeed);
        var winchSpeed = wristSpdFunction.get();
        winchSpeed = Math.abs(winchSpeed) > Constants.Joystick.DEADBAND ? winchSpeed: 0;
        elevatorSubsystem.setWinchSpeed(winchSpeed);
        if (gripperHorizontalFunction.get()) elevatorSubsystem.setGripperHorizontal();
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stop();
    }
}
