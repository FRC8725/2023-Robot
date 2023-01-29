package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.VisionManager;

import java.util.function.Supplier;


public class ElevatorJoystickCmd extends CommandBase {

    ElevatorSubsystem elevatorSubsystem;
    Supplier<Double> armSpdFunction;
    Supplier<Double> wristSpdFunction;
    Supplier<Double> winchSpdFunction;
    Supplier<Boolean> freeFunction;

    public ElevatorJoystickCmd(ElevatorSubsystem elevatorSubsystem,
                               Supplier<Double> armSpdFunction,
                               Supplier<Double> wristSpdFunction,
                               Supplier<Double> winchSpdFunction,
                               Supplier<Boolean> freeFunction) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSpdFunction = armSpdFunction;
        this.wristSpdFunction = wristSpdFunction;
        this.winchSpdFunction = winchSpdFunction;
        this.freeFunction = freeFunction;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var armSpeed = armSpdFunction.get();
        armSpeed = Math.abs(armSpeed) > Constants.Joystick.kDeadband ? armSpeed: 0;
        elevatorSubsystem.setArmSpeed(armSpeed);
        var winchSpeed = winchSpdFunction.get();
        winchSpeed = Math.abs(winchSpeed) > Constants.Joystick.kDeadband ? winchSpeed: 0;
        elevatorSubsystem.setWinchSpeed(winchSpeed);
        var wristSpeed = wristSpdFunction.get();
        wristSpeed = Math.abs(winchSpeed) > Constants.Joystick.kDeadband ? winchSpeed: 0;
        elevatorSubsystem.setWristSpeed(wristSpeed);
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
