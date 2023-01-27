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

    public ElevatorJoystickCmd(ElevatorSubsystem elevatorSubsystem,
                               Supplier<Double> elevatorSpdFunction,
                               Supplier<Double> armSpdFunction,
                               Supplier<Double> wristSpdFunction,
                               Supplier<Double> winchSpdFunction) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorSpdFunction = elevatorSpdFunction;
        this.armSpdFunction = armSpdFunction;
        this.wristSpdFunction = wristSpdFunction;
        this.winchSpdFunction = winchSpdFunction;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        var elevatorSpeed = elevatorSpdFunction.get();
<<<<<<< HEAD
        elevatorSpeed = Math.abs(elevatorSpeed) > Constants.Joystick.kDeadband ? elevatorSpeed * 0.02 : 0.0;
=======
        elevatorSpeed = Math.abs(elevatorSpeed) > Constants.Joystick.kDeadband ? elevatorSpeed * 0.03 : 0.0;
>>>>>>> b96dcd5585729ccd61259dc52c3d2246c91f1e38
        elevatorSubsystem.setElevatorSpeed(elevatorSpeed);
        var armSpeed = armSpdFunction.get();
        armSpeed = Math.abs(armSpeed) > Constants.Joystick.kDeadband ? armSpeed : 0;
        elevatorSubsystem.setArmSpeed(armSpeed);
        var wristSpeed = wristSpdFunction.get();
        wristSpeed = Math.abs(wristSpeed) > Constants.Joystick.kDeadband ? wristSpeed : 0;
        elevatorSubsystem.setWristSpeed(wristSpeed);
        var winchSpeed = wristSpdFunction.get();
        winchSpeed = Math.abs(winchSpeed) > Constants.Joystick.kDeadband ? winchSpeed : 0;
        elevatorSubsystem.setWinchSpeed(winchSpeed);
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
