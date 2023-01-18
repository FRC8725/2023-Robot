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

    public ElevatorJoystickCmd(ElevatorSubsystem elevatorSubsystem,
                               Supplier<Double> elevatorSpdFunction,
                               Supplier<Double> armSpdFunction,
                               Supplier<Double> wristSpdFunction) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorSpdFunction = elevatorSpdFunction;
        this.armSpdFunction = armSpdFunction;
        this.wristSpdFunction = wristSpdFunction;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double elevatorSpeed = elevatorSpdFunction.get();
        elevatorSpeed = Math.abs(elevatorSpeed) > Constants.Joystick.kDeadband ? elevatorSpeed : 0.0;
        elevatorSubsystem.setElevatorSpeed(elevatorSpeed);
        double armSpeed = armSpdFunction.get();
        armSpeed = Math.abs(armSpeed) > Constants.Joystick.kDeadband ? armSpeed : 0;
        elevatorSubsystem.setArmSpeed(armSpeed);
        double wristSpeed = wristSpdFunction.get();
        wristSpeed = Math.abs(wristSpeed) > Constants.Joystick.kDeadband ? wristSpeed : 0;
        elevatorSubsystem.setWristSpeed(wristSpeed);
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
