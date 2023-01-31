package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.Supplier;


public class ArmJoystickCmd extends CommandBase {

    ArmSubsystem armSubsystem;
    Supplier<Double> xSpdFunction, ySpdFunction;
    Supplier<Boolean> freeFunction;

    public ArmJoystickCmd(ArmSubsystem armSubsystem,
                          Supplier<Double> xSpdFunction,
                          Supplier<Double> ySpdFunction) {
        this.armSubsystem = armSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        var xSpeed = xSpdFunction.get();
        xSpeed = Math.abs(xSpeed) > Constants.Joystick.kDeadband ? xSpeed: 0;
        var ySpeed = ySpdFunction.get();
        ySpeed = Math.abs(ySpeed) > Constants.Joystick.kDeadband ? ySpeed: 0;
        armSubsystem.setSpeed(xSpeed, ySpeed);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.stop();
    }
}
