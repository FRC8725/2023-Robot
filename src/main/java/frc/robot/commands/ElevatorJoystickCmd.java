package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.function.Supplier;


public class ElevatorJoystickCmd extends CommandBase {

    ElevatorSubsystem elevatorSubsystem;
    Supplier<Double> xSpdFunction;
    Supplier<Double> ySpdFunction;
    Supplier<Boolean> freeFunction;

    public ElevatorJoystickCmd(ElevatorSubsystem elevatorSubsystem,
                               Supplier<Double> xSpdFunction,
                               Supplier<Double> ySpdFunction,
                               Supplier<Boolean> freeFunction) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.freeFunction = freeFunction;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var xSpeed = xSpdFunction.get();
        xSpeed = Math.abs(xSpeed) > Constants.Joystick.kDeadband ? xSpeed: 0;
        var ySpeed = ySpdFunction.get();
        ySpeed = Math.abs(xSpeed) > Constants.Joystick.kDeadband ? xSpeed: 0;
        elevatorSubsystem.setSpeed(xSpeed, ySpeed);
        elevatorSubsystem.freeControl(freeFunction.get());
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
